#pragma once
#include "../processor.h"
#include <algorithm>
#include <atomic>

// TODO: This block is useless and weird, get rid of it
namespace dsp::audio {
    class Volume : public Processor<stereo_t, stereo_t> {
        using base_type = Processor<stereo_t, stereo_t>;
    public:
        Volume() {}

        Volume(stream<stereo_t>* in, double volume, bool muted) { init(in, volume, muted); }

        void init(stream<stereo_t>* in, double volume, bool muted) {
            _volume = powf(volume, 2);
            _muted = muted;
            base_type::init(in);
        }

        void setVolume(double volume) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _volume = powf(volume, 2);
        }

        void setMuted(bool muted) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _muted = muted;
        }

        bool getMuted() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            return _muted;
        }

        // Transmit gate. While *gate reads true the output fades out quickly and
        // stays silent; once it clears, the output stays silent for a short hold
        // (relay click, receiver recovery) and then fades back in. The flag is
        // read once per block, so it can be flipped from any thread. nullptr
        // disables the gate. The flag must outlive this block.
        void setTxGateFlag(const std::atomic<bool>* gate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _txGate = gate;
        }

        void setSampleRate(double sampleRate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            if (sampleRate > 0.0) { _sampleRate = sampleRate; }
        }

        // IN-PLACE: safe in practice (in == out). Element-wise VOLK kernel; GNU
        // Radio relies on that, but VOLK does not document it as a guarantee.
        inline int process(int count, const stereo_t* in, stereo_t* out) {
            const float level = _muted ? 0.0f : _volume;
            const bool tx = _txGate && _txGate->load(std::memory_order_relaxed);

            // Fast path: gate idle and fully open
            if (!tx && _holdLeft <= 0 && _gateGain >= 1.0f) {
                volk_32f_s32f_multiply_32f((float*)out, (float*)in, level, count * 2);
                return count;
            }

            const float fadeOutStep = 1.0f / (float)(FADE_OUT_SECONDS * _sampleRate);
            const float fadeInStep = 1.0f / (float)(FADE_IN_SECONDS * _sampleRate);
            const int holdSamples = (int)(HOLD_SECONDS * _sampleRate);
            if (tx) { _holdLeft = holdSamples; }

            for (int i = 0; i < count; i++) {
                if (tx || _holdLeft > 0) {
                    if (!tx) { _holdLeft--; }
                    _gateGain = std::max(0.0f, _gateGain - fadeOutStep);
                }
                else {
                    _gateGain = std::min(1.0f, _gateGain + fadeInStep);
                }
                const float g = level * _gateGain;
                out[i].l = in[i].l * g;
                out[i].r = in[i].r * g;
            }
            return count;
        }

        virtual int run() {
            int count = base_type::_in->read();
            if (count < 0) { return -1; }

            process(count, base_type::_in->readBuf, base_type::out.writeBuf);

            base_type::_in->flush();
            if (!base_type::out.swap(count)) { return -1; }
            return count;
        }

    private:
        static constexpr double FADE_OUT_SECONDS = 0.02;
        static constexpr double HOLD_SECONDS = 0.15;
        static constexpr double FADE_IN_SECONDS = 0.2;

        float _volume;
        bool _muted;

        const std::atomic<bool>* _txGate = nullptr;
        double _sampleRate = 48000.0;
        float _gateGain = 1.0f;
        int _holdLeft = 0;

    };
}
