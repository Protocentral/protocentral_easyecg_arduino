// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, ProtoCentral Electronics
//
// Arduino library for the ProtoCentral easyECG single-lead ECG breakout board
//
// The easyECG is the easyEMG circuit with ECG component values: an INA333
// instrumentation amplifier (x11) and two OPA2333 stages form a single-supply,
// AC-coupled front-end with a total gain of about x535 and a passband of roughly
// 0.5-31 Hz. The result is digitised by an on-board TI TLA2024 12-bit I2C ADC, read
// DIFFERENTIALLY against the board's buffered 1.65 V mid-rail.
//
// The analog filter only takes about 6 dB off 50 Hz, so this library does the rest in
// software, on a fixed, host-paced sample clock:
//
//     ADC @ 1000 SPS -> 50/60 Hz notch -> 4th-order Butterworth low-pass (40 Hz)
//                    -> decimate to the output rate (125 or 250 Hz)
//                    -> optional high-pass (OFF by default - see setHighPass())
//                    -> R-peak detector -> heart rate
//
// It also watches for amplifier saturation and for excess mains pickup, which is how
// a missing electrode (lead-off) shows up on a board with no lead-off hardware.
//
// This library is fully self-contained: the TLA2024 register logic, the filters and
// the heart-rate detector are built in, so there are NO external library dependencies.
//
// For research and education only - not a medical device.
//
// https://github.com/Protocentral/protocentral_easyecg_arduino

#ifndef PROTOCENTRAL_EASYECG_H
#define PROTOCENTRAL_EASYECG_H

#include <Arduino.h>
#include <Wire.h>

/////////////////////////////////////////////////////////////////////////////////////////
// I2C address
/////////////////////////////////////////////////////////////////////////////////////////
// JP1 bridged 1-2 straps the TLA2024 ADDR pin to 3.3 V -> 0x49. Cut and bridge 2-3 for
// 0x48. An easyEMG ships at 0x48, so the two share a bus without touching the jumper.
#define EASYECG_I2C_ADDR_DEFAULT   0x49
#define EASYECG_I2C_ADDR_ALT       0x48

/////////////////////////////////////////////////////////////////////////////////////////
// Front-end model
/////////////////////////////////////////////////////////////////////////////////////////
// Total input-referred gain: INA333 x11 (R10 = 10k) times the second stage
// 1 + R14/R19 = 1 + 100k/2.1k = x48.6  ->  x535.
//
// This gain is set by the onboard resistors. It is not programmable: the only way to
// change it is to change R10, R14 or R19 on the board. Only absolute microvolts depend
// on this number - the trace shape, heart rate and lead-off detection do not.
#define EASYECG_GAIN               535.0f

// Analog passband, for reference in sketches and docs.
#define EASYECG_HPF_HZ             0.49f    // C6 1 uF / R13 324k, 1-pole
#define EASYECG_LPF_HZ             31.0f    // Sallen-Key, 33k / 100n, Q = 0.5

// Counts beyond which the analog chain is treated as saturated. The OPA2333 output
// swings to within ~50 mV of its rails, i.e. about +/-1.6 V around the 1.65 V mid-rail.
// At +/-2.048 V full-scale that is about +/-1600 counts; 1500 leaves a little margin.
#define EASYECG_SATURATION_COUNTS  1500

/////////////////////////////////////////////////////////////////////////////////////////
// TLA2024 registers
/////////////////////////////////////////////////////////////////////////////////////////
// Verified against TI SBAS846 (TLA2021/TLA2022/TLA2024, Nov 2017), Table 7.
//
// The operating word this library writes is 0x04C3:
//   [15]    OS   = 0     no effect when writing during continuous conversion
//   [14:12] MUX  = 000   AIN0 - AIN1. AIN0 is the signal, AIN1 is the buffered mid-rail.
//   [11:9]  PGA  = 010   FSR = +/-2.048 V   <- ECG range; see begin()
//   [8]     MODE = 0     continuous conversion
//   [7:5]   DR   = 110   3300 SPS
//   [4:0]   resv = 00011 datasheet requires 03h always
//
// Conversion register: D[11:0] occupies bits 15:4; an arithmetic >>4 recovers the
// signed 12-bit count.
#define EASYECG_CONV_REG           0x00
#define EASYECG_CONF_REG           0x01

// Default ADC sample clock. 8-bit AVR boards (UNO R3, Nano, Mega) cannot run the float
// filters at 1000 SPS, so they default to 500 SPS; everything else runs at 1000.
#if defined(__AVR__)
#define EASYECG_DEFAULT_ADC_RATE   500
#else
#define EASYECG_DEFAULT_ADC_RATE   1000
#endif

// Sizes of the internal buffers.
#define EASYECG_MAX_DECIMATION     16       // ADC rate / output rate
#define EASYECG_MWI_MAX            40       // 150 ms at up to 250 Hz output
#define EASYECG_RR_HISTORY         8

class easyECG
{
  public:
    /////////////////////////////////////////////////////////////////////////////////////
    // Low-level ADC enums
    /////////////////////////////////////////////////////////////////////////////////////
    enum DataRate {
        DR_128SPS  = 0x00,
        DR_250SPS  = 0x01,
        DR_490SPS  = 0x02,
        DR_920SPS  = 0x03,
        DR_1600SPS = 0x04,
        DR_2400SPS = 0x05,
        DR_3300SPS = 0x06,
    };

    enum FSRange {
        FSR_6_144V = 0x00,
        FSR_4_096V = 0x01,
        FSR_2_048V = 0x02,
        FSR_1_024V = 0x03,
        FSR_0_512V = 0x04,
        FSR_0_256V = 0x05,
    };

    enum Mux {
        MUX_AIN0_AIN1 = 0x00,
        MUX_AIN0_AIN3 = 0x01,
        MUX_AIN1_AIN3 = 0x02,
        MUX_AIN2_AIN3 = 0x03,
        MUX_AIN0_GND  = 0x04,
        MUX_AIN1_GND  = 0x05,
        MUX_AIN2_GND  = 0x06,
        MUX_AIN3_GND  = 0x07,
    };

    enum Mode {
        MODE_CONTINUOUS  = 0,
        MODE_SINGLE_SHOT = 1,
    };

    /////////////////////////////////////////////////////////////////////////////////////
    // Construction / connection
    /////////////////////////////////////////////////////////////////////////////////////
    easyECG(uint8_t i2cAddress = EASYECG_I2C_ADDR_DEFAULT);

    // Calls wirePort.begin() and configures the ADC: AIN0/AIN1, continuous, 3300 SPS,
    // +/-2.048 V. Also builds the default filter chain (1000 SPS in - 500 on AVR - 125 Hz out, 50 Hz
    // notch, 40 Hz low-pass, no high-pass).
    //
    // +/-2.048 V is matched to the x535 front end: the ADC's +/-3.8 mV input-referred
    // span sits just outside the amplifier's own +/-3.1 mV swing, so the analog stage,
    // not the ADC, sets the limit. The easyEMG's +/-0.256 V range would clip a 1 mV
    // R-wave.
    //
    // Returns true if the ADC ACKs on I2C. Does not print to Serial.
    bool begin(TwoWire &wirePort = Wire);

    bool isConnected();

    /////////////////////////////////////////////////////////////////////////////////////
    // Single unfiltered samples (each call takes a fresh reading)
    /////////////////////////////////////////////////////////////////////////////////////
    int16_t readRaw();              // signed 12-bit counts, baseline-corrected
    int16_t readRawUncorrected();   // signed 12-bit counts straight from the ADC
    float   readVoltage();          // differential ADC voltage, volts (output-referred)
    float   readMicrovolts();       // uV at the electrodes, unfiltered

    /////////////////////////////////////////////////////////////////////////////////////
    // Filtered, paced pipeline - the normal way to use the board
    /////////////////////////////////////////////////////////////////////////////////////
    // update() is non-blocking. Call it as often as you can from loop(). It takes one ADC
    // sample whenever the sample clock is due, runs the filters, and returns true when a
    // new OUTPUT sample is ready - at outputRateHz(). Read it with the accessors below.
    //
    // The sample clock is fixed rather than free-running because the notch and low-pass
    // coefficients are built for an exact rate. A free-running loop's rate wanders by
    // 10% or more, which moves a Q=4 notch off mains entirely.
    bool    update();

    // ADC rate and output rate. adcRateHz must be an integer multiple of outputRateHz,
    // at most EASYECG_MAX_DECIMATION times it. Rebuilds every filter. Returns false and
    // leaves the old setting if the pair is invalid.
    //
    // 1000 / 125 suits OpenView; 1000 / 250 suits plotting. On an 8-bit AVR (UNO R3)
    // use 500 / 125 - the float filters will not keep up with 1000 SPS there.
    bool     setSampling(uint16_t adcRateHz, uint16_t outputRateHz);
    uint16_t adcRateHz();
    uint16_t outputRateHz();

    // Mains notch, 50 or 60 Hz. On by default at 50 Hz. Pass 0 to disable.
    // The notch is required, not optional, for normal use: the analog low-pass alone
    // only takes about 6 dB off 50 Hz.
    bool    setNotch(float mainsHz, float q = 4.0f);
    bool    notchEnabled();

    // Digital low-pass corner, Hz (4th-order Butterworth at the ADC rate). Default 40.
    // Pass 0 to disable (the analog Sallen-Key still band-limits to ~31 Hz).
    // Going below ~25 Hz makes the trace smoother but rounds and shrinks the QRS.
    bool    setLowPass(float cornerHz);

    // Digital high-pass corner, Hz (1-pole, at the output rate). Default 0 = OFF.
    //
    // It is off by default on purpose. The board is already AC-coupled at ~0.5 Hz, and
    // every extra high-pass pole distorts the ST segment - a depressed or elevated ST
    // is real physiology and must not be filtered into or out of existence. Turn it on
    // (0.5 Hz) only if baseline wander bothers you more than ST fidelity.
    void    setHighPass(float cornerHz);

    // Output-rate sample accessors (valid after update() returns true).
    int16_t ecgFiltered();           // counts, filtered
    float   ecgMicrovolts();         // uV at the electrodes, filtered
    int16_t ecgUnfiltered();         // counts, plain average of the ADC samples
    float   ecgUnfilteredMicrovolts();

    // Samples the sketch could not take on time because loop() was too slow. If this
    // grows, lower the ADC rate or shorten what loop() does between update() calls.
    uint32_t missedSamples();

    // Clears all filter, detector and status state - e.g. after re-attaching electrodes.
    void    resetFilters();

    /////////////////////////////////////////////////////////////////////////////////////
    // Heart rate (simplified Pan-Tompkins on the filtered output)
    /////////////////////////////////////////////////////////////////////////////////////
    // Needs ~2 s to learn the signal level, then a few beats to average.
    float   heartRate();             // beats per minute, 0 until established
    bool    beatDetected();          // true once per detected beat; clears when read
    uint16_t lastRRMillis();         // most recent R-R interval, ms

    /////////////////////////////////////////////////////////////////////////////////////
    // Signal-quality status
    /////////////////////////////////////////////////////////////////////////////////////
    // Saturated: the amplifier is pinned near a rail for most of the last ~250 ms. On a
    // mains-powered host this is the driven-right-leg failure - switch to battery or an
    // isolated Qwiic adapter (easyISO). It is also what a floating electrode looks like.
    bool    isSaturated();

    // Mains pickup, uV RMS at the electrodes - what the notch is removing.
    float   mainsMicrovolts();

    // Lead-off heuristic: saturated, or mains pickup above the threshold.
    // PROVISIONAL: the default threshold has not yet been validated on the bench for
    // each electrode. Measure your own (05.Diagnostics) before relying on it.
    bool    isLeadOff();
    void    setLeadOffThreshold(float mainsMicrovoltsRMS);
    float   getLeadOffThreshold();

    /////////////////////////////////////////////////////////////////////////////////////
    // Baseline (zeroing)
    /////////////////////////////////////////////////////////////////////////////////////
    // Averages the uncorrected reading over durationMs - long enough to span whole
    // heartbeats, so the R-waves do not bias the result. Electrodes attached, subject
    // still.
    void    calibrateBaseline(uint16_t durationMs = 1000);
    void    setBaseline(int16_t offsetCounts);
    int16_t getBaseline();

    /////////////////////////////////////////////////////////////////////////////////////
    // Optional / advanced low-level passthroughs
    /////////////////////////////////////////////////////////////////////////////////////
    void    setDataRate(DataRate rate);
    void    setRange(FSRange range);
    void    setMode(Mode mode);
    void    setMux(Mux mux);
    float   fullScaleVolts();
    uint16_t readConfig();           // CONF register read-back
    int16_t readSingleShot();

  private:
    struct Biquad {
        float b0, b1, b2, a1, a2;
        float z1, z2;
    };

    void     writeReg(uint8_t reg, uint16_t data);
    uint16_t readReg(uint8_t reg);
    uint16_t confWord();
    void     applyConf();
    float    countsToMicrovolts(float counts);

    void     rebuildFilters();
    static void  designLowPass(Biquad &f, float fc, float fs, float q);
    static void  designNotch(Biquad &f, float f0, float fs, float q);
    static float runBiquad(Biquad &f, float x);
    static void  clearBiquad(Biquad &f);

    void     processAdcSample(int16_t uncorrected);
    void     processOutputSample(float x);
    void     detectBeat(float x);

    TwoWire *_wire;
    uint8_t  _i2cAddr;

    uint8_t  _mux, _fsr, _mode, _dr;

    int16_t  _baseline;

    // Sampling.
    uint16_t _adcRate;
    uint16_t _outRate;
    uint8_t  _decim;
    uint32_t _periodUs;
    uint32_t _nextUs;
    bool     _clockRunning;
    uint32_t _missed;

    // Filters. The notch is always run - its residue is the mains measurement - and
    // _notchOn decides whether its output is used.
    bool     _notchOn;
    float    _notchHz, _notchQ;
    float    _lpfHz;
    float    _hpfHz;
    Biquad   _notch;
    Biquad   _lpf1, _lpf2;
    float    _hpfA, _hpfPrevX, _hpfPrevY;

    // Decimation.
    uint8_t  _decimCount;
    float    _rawAccum;
    float    _lastFilteredAdc;
    bool     _outReady;
    float    _outFiltered;
    float    _outUnfiltered;

    // Status.
    float    _satLevel;              // EMA of "sample beyond saturation threshold"
    float    _satAlpha;
    float    _mainsSq;               // EMA of squared notch residue, counts^2
    float    _mainsAlpha;
    float    _leadOffUv;

    // Heart-rate detector (runs at the output rate).
    float    _hrX[5];                // derivative history
    float    _mwiBuf[EASYECG_MWI_MAX];
    uint8_t  _mwiLen, _mwiIdx;
    float    _mwiSum;
    float    _sigLevel, _noiseLevel;
    uint32_t _hrSampleCount;
    uint32_t _lastBeatSample;
    bool     _haveBeat;
    bool     _inPeak;
    float    _peakVal;
    uint16_t _rr[EASYECG_RR_HISTORY];
    uint8_t  _rrCount, _rrIdx;
    uint16_t _lastRR;
    float    _bpm;
    bool     _beatFlag;

    union I2C_data {
        uint8_t  u8_bytes[2];
        uint16_t u16_value;
    } _i2c_data;
};

#endif // PROTOCENTRAL_EASYECG_H
