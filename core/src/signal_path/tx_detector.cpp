#include "tx_detector.h"
#include "signal_path.h"
#include <algorithm>
#include <cmath>
#include <dsp/window/hann.h>
#include <utils/flog.h>

TxDetector::~TxDetector() {
    stop();
}

void TxDetector::start() {
    if (running) { return; }

    if (!plan.get()) {
        fftIn.allocate(FFT_SIZE);
        fftOut.allocate(FFT_SIZE);
        plan.planDFT1D(FFT_SIZE, fftIn, fftOut, FFTW_FORWARD);
        window.resize(FFT_SIZE);
        for (int i = 0; i < FFT_SIZE; i++) {
            window[i] = (float)dsp::window::hann(i, FFT_SIZE);
        }
        binPower.reserve(FFT_SIZE);
    }
    fill = 0;
    skipLeft = 0.0;
    relearnRequested.store(false, std::memory_order_relaxed);
    resetState();

    // The stream and the sink are set up once and reused: init() registers
    // the stream as an input of the block every time it is called, so
    // re-initialising with a fresh stream would leave the freed one in the
    // block's input list and crash the next stop().
    if (!iqStream) {
        iqStream = std::make_unique<dsp::stream<dsp::complex_t>>();
        sink.init(iqStream.get(), handler, this);
    }
    sink.start();
    sigpath::iqFrontEnd.bindIQStream(iqStream.get());
    running = true;
    flog::info("[TxDetector] Started");
}

void TxDetector::stop() {
    if (!running) { return; }
    sigpath::iqFrontEnd.unbindIQStream(iqStream.get());
    sink.stop();
    running = false;
    setTx(false);
    flog::info("[TxDetector] Stopped");
}

void TxDetector::resetState() {
    smoothedValid = false;
    refValid = false;
    txSeconds = 0.0;
    settleLeft = 0.0;
    confirm = 0;
    referenceValid.store(false, std::memory_order_relaxed);
}

void TxDetector::setTx(bool tx) {
    if (transmitting.exchange(tx, std::memory_order_relaxed) == tx) { return; }
    flog::info("[TxDetector] {0}", tx ? "TX" : "RX");
    sigpath::txState.emit(tx);
}

void TxDetector::handler(dsp::complex_t* data, int count, void* ctx) {
    TxDetector* _this = (TxDetector*)ctx;
    double sampleRate = sigpath::iqFrontEnd.getEffectiveSamplerate();
    if (sampleRate <= 0.0) { return; }

    int i = 0;
    while (i < count) {
        // Skip ahead to the next frame
        if (_this->skipLeft > 0.0) {
            int skip = std::min<int>(count - i, (int)std::ceil(_this->skipLeft));
            _this->skipLeft -= skip;
            i += skip;
            continue;
        }

        // Fill the frame (windowed)
        int n = std::min(count - i, FFT_SIZE - _this->fill);
        fftwf_complex* in = _this->fftIn.get();
        for (int j = 0; j < n; j++) {
            float w = _this->window[_this->fill + j];
            in[_this->fill + j][0] = data[i + j].re * w;
            in[_this->fill + j][1] = data[i + j].im * w;
        }
        _this->fill += n;
        i += n;

        if (_this->fill == FFT_SIZE) {
            _this->fill = 0;
            _this->skipLeft = std::max(0.0, FRAME_INTERVAL * sampleRate - FFT_SIZE);
            _this->processFrame();
        }
    }
}

void TxDetector::processFrame() {
    double sampleRate = sigpath::iqFrontEnd.getEffectiveSamplerate();
    double frameSeconds = std::max(FRAME_INTERVAL, FFT_SIZE / sampleRate);

    if (relearnRequested.exchange(false, std::memory_order_relaxed)) {
        resetState();
        setTx(false);
    }

    fftwf_execute(plan);

    // Noise floor: median bin power, leaving out the DC spike and the band
    // edges where the SDR's anti-alias filter rolls off.
    binPower.clear();
    const fftwf_complex* out = fftOut.get();
    const int dcGuard = 4;
    const int edge = (int)(FFT_SIZE * 0.4);
    for (int k = 0; k < FFT_SIZE; k++) {
        int f = (k < FFT_SIZE / 2) ? k : k - FFT_SIZE;
        int af = std::abs(f);
        if (af <= dcGuard || af > edge) { continue; }
        binPower.push_back(out[k][0] * out[k][0] + out[k][1] * out[k][1]);
    }
    auto mid = binPower.begin() + binPower.size() / 2;
    std::nth_element(binPower.begin(), mid, binPower.end());
    double frameDb = 10.0 * std::log10(std::max<double>(*mid, 1e-30)) - 20.0 * std::log10((double)FFT_SIZE);

    // Light smoothing of the estimate
    if (!smoothedValid) {
        smoothedDb = frameDb;
        smoothedValid = true;
    }
    else {
        double a = std::exp(-frameSeconds / FLOOR_TAU);
        smoothedDb = a * smoothedDb + (1.0 - a) * frameDb;
    }
    floorDb.store((float)smoothedDb, std::memory_order_relaxed);

    if (!refValid) {
        refDb = smoothedDb;
        refValid = true;
        referenceDb.store((float)refDb, std::memory_order_relaxed);
        referenceValid.store(true, std::memory_order_relaxed);
        return;
    }

    const double threshold = thresholdDb.load(std::memory_order_relaxed);
    const double drop = refDb - smoothedDb;

    if (transmitting.load(std::memory_order_relaxed)) {
        txSeconds += frameSeconds;
        confirm = (drop < threshold / 2.0) ? confirm + 1 : 0;
        if (confirm >= CONFIRM_FRAMES) {
            confirm = 0;
            settleLeft = SETTLE_SECONDS;
            setTx(false);
        }
        else if (txSeconds >= MAX_TX_SECONDS) {
            // A drop this long is more likely a gain or antenna change than TX:
            // accept the new level instead of staying muted.
            confirm = 0;
            refDb = smoothedDb;
            setTx(false);
        }
    }
    else {
        confirm = (drop > threshold) ? confirm + 1 : 0;
        if (confirm >= CONFIRM_FRAMES) {
            confirm = 0;
            txSeconds = 0.0;
            setTx(true);
        }
        else if (confirm == 0) {
            if (settleLeft > 0.0) {
                settleLeft -= frameSeconds;
            }
            else {
                // Follow the floor up quickly, but down only slowly: a drop
                // smaller than the threshold (quiet band, small TX contrast)
                // must not drag the reference down during a transmission, or
                // the drop vanishes and TX is never detected.
                double tau = (drop > REFERENCE_DEADBAND_DB) ? REFERENCE_DOWN_TAU : REFERENCE_TAU;
                double a = std::exp(-frameSeconds / tau);
                refDb = a * refDb + (1.0 - a) * smoothedDb;
            }
        }
    }
    referenceDb.store((float)refDb, std::memory_order_relaxed);
}
