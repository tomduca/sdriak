#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include <dsp/stream.h>
#include <dsp/types.h>
#include <dsp/sink/handler_sink.h>
#include <dsp/buffer/fftw.h>

// Detects transmit on a panadapter fed from a transceiver's IF, without CAT.
//
// While the radio transmits, its receive chain is cut off from the antenna, so
// the band noise disappears from the IF: the panadapter's noise floor drops by
// well over 10 dB across the whole span, while the radio's own signal and a few
// spurs remain. Total power is useless here (the TX lines make up for the lost
// noise), so the detector measures the noise floor as the median of an FFT's
// bins, which ignores the lines, and tracks its normal receive level. A drop
// of more than `threshold` dB below that level is TX; coming back within half
// the threshold is RX again. The floor stays down for the whole transmission,
// including pauses in SSB speech.
//
// On every change it emits sigpath::txState, which mutes the audio outputs,
// holds the demodulator AGC and freezes noise reduction.
class TxDetector {
public:
    TxDetector() {}
    ~TxDetector();

    // Bind to the IQ front end and start measuring. Safe to call repeatedly.
    void start();
    // Stop measuring; emits txState(false) if TX was active.
    void stop();
    bool isRunning() const { return running; }

    // Forget the learned receive floor and go back to RX (e.g. after changing
    // the SDR gain, which moves the floor just like TX does).
    void relearn() { relearnRequested.store(true, std::memory_order_relaxed); }

    void setThreshold(float dropDb) { thresholdDb.store(dropDb, std::memory_order_relaxed); }
    float getThreshold() const { return thresholdDb.load(std::memory_order_relaxed); }

    // Live readouts for the GUI (dB, relative)
    float getFloorDb() const { return floorDb.load(std::memory_order_relaxed); }
    float getReferenceDb() const { return referenceDb.load(std::memory_order_relaxed); }
    bool hasReference() const { return referenceValid.load(std::memory_order_relaxed); }
    bool isTransmitting() const { return transmitting.load(std::memory_order_relaxed); }

private:
    static void handler(dsp::complex_t* data, int count, void* ctx);
    void processFrame();
    void setTx(bool tx);
    void resetState();

    static constexpr int FFT_SIZE = 1024;
    static constexpr double FRAME_INTERVAL = 0.02;   // one floor estimate every 20 ms
    static constexpr double FLOOR_TAU = 0.04;        // smoothing of the floor estimate
    static constexpr double REFERENCE_TAU = 2.0;     // tracking of the receive floor (up)
    static constexpr double REFERENCE_DOWN_TAU = 60.0; // tracking when the floor sits below it
    static constexpr double REFERENCE_DEADBAND_DB = 1.5; // normal floor jitter around the reference
    static constexpr double SETTLE_SECONDS = 1.0;    // no reference tracking right after TX
    static constexpr double MAX_TX_SECONDS = 180.0;  // longer "TX" is a level change, accept it
    static constexpr int CONFIRM_FRAMES = 2;         // frames beyond the threshold to switch

    bool running = false;
    std::unique_ptr<dsp::stream<dsp::complex_t>> iqStream;
    dsp::sink::Handler<dsp::complex_t> sink;

    // Worker-thread state
    dsp::buffer::FFTWBuffer<fftwf_complex> fftIn, fftOut;
    dsp::buffer::FFTWPlan plan;
    std::vector<float> window;
    std::vector<float> binPower;
    int fill = 0;
    double skipLeft = 0.0;
    bool smoothedValid = false;
    double smoothedDb = 0.0;
    double refDb = 0.0;
    bool refValid = false;
    double txSeconds = 0.0;
    double settleLeft = 0.0;
    int confirm = 0;

    // Shared with the GUI
    std::atomic<float> thresholdDb{ 10.0f };
    std::atomic<float> floorDb{ 0.0f };
    std::atomic<float> referenceDb{ 0.0f };
    std::atomic<bool> referenceValid{ false };
    std::atomic<bool> transmitting{ false };
    std::atomic<bool> relearnRequested{ false };
};
