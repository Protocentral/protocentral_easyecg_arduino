// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, ProtoCentral Electronics
//
// Arduino library for the ProtoCentral easyECG single-lead ECG breakout board
//
// https://github.com/Protocentral/protocentral_easyecg_arduino

#include "ProtoCentral_easyECG.h"

#define EASYECG_CONF_COMP_DEFAULT  0x03
#define EASYECG_PI                 3.14159265358979f

// Butterworth 4th order as two cascaded 2nd-order sections.
#define EASYECG_BW4_Q1             0.54119610f
#define EASYECG_BW4_Q2             1.30656296f

// Default lead-off mains threshold, uV RMS at the electrodes. A good contact picks up a
// few uV of mains; a floating input picks up orders of magnitude more. PROVISIONAL -
// not yet validated on the bench per electrode.
#define EASYECG_DEFAULT_LEADOFF_UV 200.0f

static const float EASYECG_FSR_VOLTS[6] = {
    6.144f, 4.096f, 2.048f, 1.024f, 0.512f, 0.256f,
};

easyECG::easyECG(uint8_t i2cAddress)
{
    _wire    = &Wire;
    _i2cAddr = i2cAddress;

    _mux  = MUX_AIN0_AIN1;
    _fsr  = FSR_2_048V;
    _mode = MODE_CONTINUOUS;
    _dr   = DR_3300SPS;

    _baseline = 0;

    _adcRate  = EASYECG_DEFAULT_ADC_RATE;
    _outRate  = 125;
    _decim    = (uint8_t)(EASYECG_DEFAULT_ADC_RATE / 125);
    _periodUs = 1000000UL / EASYECG_DEFAULT_ADC_RATE;
    _nextUs   = 0;
    _clockRunning = false;
    _missed   = 0;

    _notchOn = true;
    _notchHz = 50.0f;
    _notchQ  = 4.0f;
    _lpfHz   = 40.0f;
    _hpfHz   = 0.0f;

    _leadOffUv = EASYECG_DEFAULT_LEADOFF_UV;

    rebuildFilters();
}

/////////////////////////////////////////////////////////////////////////////////////////
// Low-level register I/O (MSB first), ported from the ProtoCentral TLA20xx driver.
/////////////////////////////////////////////////////////////////////////////////////////

void easyECG::writeReg(uint8_t reg, uint16_t data)
{
    _i2c_data.u16_value = data;

    _wire->beginTransmission(_i2cAddr);
    _wire->write(reg);
    _wire->write(_i2c_data.u8_bytes[1]);
    _wire->write(_i2c_data.u8_bytes[0]);
    _wire->endTransmission();
}

uint16_t easyECG::readReg(uint8_t reg)
{
    _wire->beginTransmission(_i2cAddr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_i2cAddr, (uint8_t)2);

    _i2c_data.u8_bytes[1] = _wire->read();
    _i2c_data.u8_bytes[0] = _wire->read();

    return _i2c_data.u16_value;
}

uint16_t easyECG::confWord()
{
    return (uint16_t)(((_mux  & 0x07) << 12) |
                      ((_fsr  & 0x07) << 9)  |
                      ((_mode & 0x01) << 8)  |
                      ((_dr   & 0x07) << 5)  |
                      EASYECG_CONF_COMP_DEFAULT);
}

void easyECG::applyConf()
{
    writeReg(EASYECG_CONF_REG, confWord());
}

float easyECG::fullScaleVolts()
{
    return (_fsr < 6) ? EASYECG_FSR_VOLTS[_fsr] : EASYECG_FSR_VOLTS[FSR_2_048V];
}

uint16_t easyECG::readConfig()
{
    return readReg(EASYECG_CONF_REG);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Construction / connection
/////////////////////////////////////////////////////////////////////////////////////////

bool easyECG::begin(TwoWire &wirePort)
{
    _wire = &wirePort;
    _wire->begin();

    // AIN0/AIN1, +/-2.048 V, continuous, 3300 SPS -> CONF 0x04C3.
    _mux  = MUX_AIN0_AIN1;
    _fsr  = FSR_2_048V;
    _mode = MODE_CONTINUOUS;
    _dr   = DR_3300SPS;
    applyConf();

    resetFilters();
    return isConnected();
}

bool easyECG::isConnected()
{
    _wire->beginTransmission(_i2cAddr);
    return (_wire->endTransmission() == 0);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Single unfiltered samples
/////////////////////////////////////////////////////////////////////////////////////////

int16_t easyECG::readRawUncorrected()
{
    int16_t v = (int16_t)readReg(EASYECG_CONV_REG);
    v >>= 4;
    return v;
}

int16_t easyECG::readRaw()
{
    return (int16_t)(readRawUncorrected() - _baseline);
}

float easyECG::readVoltage()
{
    return readRaw() * (fullScaleVolts() / 2048.0f);
}

// counts -> microvolts at the electrodes:  uV = counts * (FS / 2048) * 1e6 / gain
float easyECG::countsToMicrovolts(float counts)
{
    return counts * (fullScaleVolts() / 2048.0f) * 1000000.0f / EASYECG_GAIN;
}

float easyECG::readMicrovolts()
{
    return countsToMicrovolts((float)readRaw());
}

/////////////////////////////////////////////////////////////////////////////////////////
// Filter design (RBJ cookbook biquads, direct form II transposed)
/////////////////////////////////////////////////////////////////////////////////////////

void easyECG::clearBiquad(Biquad &f)
{
    f.z1 = 0.0f;
    f.z2 = 0.0f;
}

void easyECG::designLowPass(Biquad &f, float fc, float fs, float q)
{
    float w0    = 2.0f * EASYECG_PI * fc / fs;
    float cosw0 = cosf(w0);
    float alpha = sinf(w0) / (2.0f * q);
    float a0    = 1.0f + alpha;

    f.b0 = ((1.0f - cosw0) * 0.5f) / a0;
    f.b1 = (1.0f - cosw0) / a0;
    f.b2 = f.b0;
    f.a1 = (-2.0f * cosw0) / a0;
    f.a2 = (1.0f - alpha) / a0;
    clearBiquad(f);
}

void easyECG::designNotch(Biquad &f, float f0, float fs, float q)
{
    float w0    = 2.0f * EASYECG_PI * f0 / fs;
    float cosw0 = cosf(w0);
    float alpha = sinf(w0) / (2.0f * q);
    float a0    = 1.0f + alpha;

    f.b0 = 1.0f / a0;
    f.b1 = (-2.0f * cosw0) / a0;
    f.b2 = f.b0;
    f.a1 = f.b1;
    f.a2 = (1.0f - alpha) / a0;
    clearBiquad(f);
}

float easyECG::runBiquad(Biquad &f, float x)
{
    float y = (f.b0 * x) + f.z1;
    f.z1 = (f.b1 * x) - (f.a1 * y) + f.z2;
    f.z2 = (f.b2 * x) - (f.a2 * y);
    return y;
}

void easyECG::rebuildFilters()
{
    float fs = (float)_adcRate;

    // The notch is always designed, even when its output is not used, because its
    // residue is the mains measurement. With the notch off, measure at 50 Hz.
    float nf = (_notchHz > 0.0f) ? _notchHz : 50.0f;
    designNotch(_notch, nf, fs, _notchQ);

    if (_lpfHz > 0.0f) {
        designLowPass(_lpf1, _lpfHz, fs, EASYECG_BW4_Q1);
        designLowPass(_lpf2, _lpfHz, fs, EASYECG_BW4_Q2);
    }

    // 1-pole high-pass at the output rate: y = a * (y' + x - x').
    if (_hpfHz > 0.0f) {
        float rc = 1.0f / (2.0f * EASYECG_PI * _hpfHz);
        float dt = 1.0f / (float)_outRate;
        _hpfA = rc / (rc + dt);
    } else {
        _hpfA = 1.0f;
    }

    // Status time constants: saturation over ~250 ms, mains over ~500 ms.
    _satAlpha   = 1.0f / (0.25f * fs);
    _mainsAlpha = 1.0f / (0.5f * fs);

    // Moving-window integrator for the beat detector: 150 ms.
    uint8_t len = (uint8_t)((0.15f * (float)_outRate) + 0.5f);
    if (len < 1) len = 1;
    if (len > EASYECG_MWI_MAX) len = EASYECG_MWI_MAX;
    _mwiLen = len;

    resetFilters();
}

void easyECG::resetFilters()
{
    clearBiquad(_notch);
    clearBiquad(_lpf1);
    clearBiquad(_lpf2);
    _hpfPrevX = 0.0f;
    _hpfPrevY = 0.0f;

    _decimCount      = 0;
    _rawAccum        = 0.0f;
    _lastFilteredAdc = 0.0f;
    _outReady        = false;
    _outFiltered     = 0.0f;
    _outUnfiltered   = 0.0f;

    _satLevel = 0.0f;
    _mainsSq  = 0.0f;

    for (uint8_t i = 0; i < 5; i++) _hrX[i] = 0.0f;
    for (uint8_t i = 0; i < EASYECG_MWI_MAX; i++) _mwiBuf[i] = 0.0f;
    _mwiIdx         = 0;
    _mwiSum         = 0.0f;
    _sigLevel       = 0.0f;
    _noiseLevel     = 0.0f;
    _hrSampleCount  = 0;
    _lastBeatSample = 0;
    _haveBeat       = false;
    _inPeak         = false;
    _peakVal        = 0.0f;
    for (uint8_t i = 0; i < EASYECG_RR_HISTORY; i++) _rr[i] = 0;
    _rrCount  = 0;
    _rrIdx    = 0;
    _lastRR   = 0;
    _bpm      = 0.0f;
    _beatFlag = false;

    _clockRunning = false;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Filter configuration
/////////////////////////////////////////////////////////////////////////////////////////

bool easyECG::setSampling(uint16_t adcRateHz, uint16_t outputRateHz)
{
    if (adcRateHz == 0 || outputRateHz == 0 || (adcRateHz % outputRateHz) != 0) {
        return false;
    }
    uint16_t d = adcRateHz / outputRateHz;
    if (d < 1 || d > EASYECG_MAX_DECIMATION) {
        return false;
    }
    // Keep the existing filter corners meaningful at the new rate.
    if (_lpfHz > 0.0f && _lpfHz > 0.45f * (float)adcRateHz) {
        return false;
    }
    if (_notchHz > 0.0f && (float)adcRateHz < _notchHz * 2.5f) {
        return false;
    }

    _adcRate  = adcRateHz;
    _outRate  = outputRateHz;
    _decim    = (uint8_t)d;
    _periodUs = 1000000UL / adcRateHz;
    rebuildFilters();
    return true;
}

uint16_t easyECG::adcRateHz()
{
    return _adcRate;
}

uint16_t easyECG::outputRateHz()
{
    return _outRate;
}

bool easyECG::setNotch(float mainsHz, float q)
{
    if (mainsHz <= 0.0f) {
        _notchOn = false;
        _notchHz = 0.0f;
        rebuildFilters();
        return true;
    }
    if ((float)_adcRate < mainsHz * 2.5f || q <= 0.0f) {
        return false;
    }
    _notchOn = true;
    _notchHz = mainsHz;
    _notchQ  = q;
    rebuildFilters();
    return true;
}

bool easyECG::notchEnabled()
{
    return _notchOn;
}

bool easyECG::setLowPass(float cornerHz)
{
    if (cornerHz < 0.0f || cornerHz > 0.45f * (float)_adcRate) {
        return false;
    }
    _lpfHz = cornerHz;
    rebuildFilters();
    return true;
}

void easyECG::setHighPass(float cornerHz)
{
    _hpfHz = (cornerHz > 0.0f) ? cornerHz : 0.0f;
    rebuildFilters();
}

/////////////////////////////////////////////////////////////////////////////////////////
// Paced pipeline
/////////////////////////////////////////////////////////////////////////////////////////

bool easyECG::update()
{
    uint32_t now = micros();

    if (!_clockRunning) {
        _clockRunning = true;
        _nextUs = now;
    }

    if ((int32_t)(now - _nextUs) < 0) {
        return false;
    }

    // Advance by whole periods so the rate stays exact. Resync only if we have fallen
    // badly behind, and count what was lost.
    uint32_t late = now - _nextUs;
    if (late > 4UL * _periodUs) {
        _missed += late / _periodUs;
        _nextUs = now;
    }
    _nextUs += _periodUs;

    _outReady = false;
    processAdcSample(readRawUncorrected());

    if (_outReady) {
        _outReady = false;
        return true;
    }
    return false;
}

void easyECG::processAdcSample(int16_t uncorrected)
{
    // Saturation: fraction of recent samples near a rail.
    int16_t mag = (uncorrected < 0) ? (int16_t)-uncorrected : uncorrected;
    float   ind = (mag >= EASYECG_SATURATION_COUNTS) ? 1.0f : 0.0f;
    _satLevel += _satAlpha * (ind - _satLevel);

    float x = (float)(uncorrected - _baseline);

    // Notch, and its residue as the mains measurement.
    float n     = runBiquad(_notch, x);
    float resid = x - n;
    _mainsSq += _mainsAlpha * ((resid * resid) - _mainsSq);

    float y = _notchOn ? n : x;

    if (_lpfHz > 0.0f) {
        y = runBiquad(_lpf1, y);
        y = runBiquad(_lpf2, y);
    }

    _rawAccum += x;
    _decimCount++;

    if (_decimCount >= _decim) {
        _outUnfiltered = _rawAccum / (float)_decim;
        _rawAccum   = 0.0f;
        _decimCount = 0;
        processOutputSample(y);
    }
}

void easyECG::processOutputSample(float x)
{
    if (_hpfHz > 0.0f) {
        float y = _hpfA * (_hpfPrevY + x - _hpfPrevX);
        _hpfPrevX = x;
        _hpfPrevY = y;
        x = y;
    }

    _outFiltered = x;
    _outReady    = true;

    detectBeat(x);
}

static int16_t clampCounts(float v)
{
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)(v >= 0.0f ? (v + 0.5f) : (v - 0.5f));
}

int16_t easyECG::ecgFiltered()
{
    return clampCounts(_outFiltered);
}

float easyECG::ecgMicrovolts()
{
    return countsToMicrovolts(_outFiltered);
}

int16_t easyECG::ecgUnfiltered()
{
    return clampCounts(_outUnfiltered);
}

float easyECG::ecgUnfilteredMicrovolts()
{
    return countsToMicrovolts(_outUnfiltered);
}

uint32_t easyECG::missedSamples()
{
    return _missed;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Heart-rate detector
/////////////////////////////////////////////////////////////////////////////////////////
// Simplified Pan-Tompkins: 5-point derivative -> square -> 150 ms moving-window
// integral -> adaptive threshold between a signal level and a noise level, with a
// 200 ms refractory period and T-wave rejection inside 360 ms.

void easyECG::detectBeat(float x)
{
    // Derivative, (2x[n] + x[n-1] - x[n-3] - 2x[n-4]) / 8.
    _hrX[4] = _hrX[3];
    _hrX[3] = _hrX[2];
    _hrX[2] = _hrX[1];
    _hrX[1] = _hrX[0];
    _hrX[0] = x;
    float d  = ((2.0f * _hrX[0]) + _hrX[1] - _hrX[3] - (2.0f * _hrX[4])) * 0.125f;
    float sq = d * d;

    // Moving-window integral.
    _mwiSum -= _mwiBuf[_mwiIdx];
    _mwiBuf[_mwiIdx] = sq;
    _mwiSum += sq;
    _mwiIdx++;
    if (_mwiIdx >= _mwiLen) _mwiIdx = 0;
    if (_mwiSum < 0.0f) _mwiSum = 0.0f;   // float drift
    float mwi = _mwiSum / (float)_mwiLen;

    _hrSampleCount++;
    uint32_t learn = 2UL * _outRate;

    // Learning phase: signal level = largest integral seen, noise = its mean.
    if (_hrSampleCount <= learn) {
        if (mwi > _sigLevel) _sigLevel = mwi;
        _noiseLevel += (mwi - _noiseLevel) / (float)_hrSampleCount;
        return;
    }

    float    threshold  = _noiseLevel + 0.25f * (_sigLevel - _noiseLevel);
    uint32_t sinceBeat  = _hrSampleCount - _lastBeatSample;
    uint32_t refractory = (uint32_t)(0.2f * _outRate);

    if (!_inPeak) {
        if (mwi > threshold && (!_haveBeat || sinceBeat > refractory)) {
            _inPeak  = true;
            _peakVal = mwi;
        } else {
            _noiseLevel += 0.01f * (mwi - _noiseLevel);
        }
    } else {
        if (mwi > _peakVal) {
            _peakVal = mwi;
        } else if (mwi < 0.5f * _peakVal) {
            _inPeak = false;

            uint16_t rrMs = (uint16_t)((sinceBeat * 1000UL) / _outRate);

            // A small peak soon after a beat is a T-wave, not a QRS.
            if (_haveBeat && rrMs < 360 && _peakVal < 0.5f * _sigLevel) {
                _noiseLevel += 0.125f * (_peakVal - _noiseLevel);
                return;
            }

            _sigLevel = 0.125f * _peakVal + 0.875f * _sigLevel;

            if (_haveBeat && rrMs >= 270 && rrMs <= 2000) {   // 30-220 bpm
                _lastRR = rrMs;
                _rr[_rrIdx] = rrMs;
                _rrIdx = (uint8_t)((_rrIdx + 1) % EASYECG_RR_HISTORY);
                if (_rrCount < EASYECG_RR_HISTORY) _rrCount++;

                uint32_t sum = 0;
                for (uint8_t i = 0; i < _rrCount; i++) sum += _rr[i];
                _bpm = 60000.0f / ((float)sum / (float)_rrCount);
            }

            _haveBeat       = true;
            _lastBeatSample = _hrSampleCount;
            _beatFlag       = true;
        }
    }

    // No beat for 3 s: the signal level is probably stale (electrode moved, amplitude
    // dropped). Let it decay so the threshold can find the beats again. After 5 s,
    // stop reporting a rate.
    if (_haveBeat && sinceBeat > 3UL * _outRate && !_inPeak) {
        _sigLevel *= 0.98f;
        if (_sigLevel < _noiseLevel) _sigLevel = _noiseLevel;
    }
    if (_haveBeat && sinceBeat > 5UL * _outRate) {
        _bpm     = 0.0f;
        _rrCount = 0;
        _rrIdx   = 0;
    }
}

float easyECG::heartRate()
{
    return _bpm;
}

bool easyECG::beatDetected()
{
    bool b = _beatFlag;
    _beatFlag = false;
    return b;
}

uint16_t easyECG::lastRRMillis()
{
    return _lastRR;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Signal-quality status
/////////////////////////////////////////////////////////////////////////////////////////

bool easyECG::isSaturated()
{
    return _satLevel > 0.5f;
}

float easyECG::mainsMicrovolts()
{
    return countsToMicrovolts(sqrtf(_mainsSq));
}

bool easyECG::isLeadOff()
{
    return isSaturated() || (_leadOffUv > 0.0f && mainsMicrovolts() > _leadOffUv);
}

void easyECG::setLeadOffThreshold(float mainsMicrovoltsRMS)
{
    _leadOffUv = mainsMicrovoltsRMS;
}

float easyECG::getLeadOffThreshold()
{
    return _leadOffUv;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Baseline (zeroing)
/////////////////////////////////////////////////////////////////////////////////////////

void easyECG::calibrateBaseline(uint16_t durationMs)
{
    if (durationMs == 0) {
        return;
    }

    // Average the UNCORRECTED count so repeated calls converge instead of stacking.
    int32_t  sum = 0;
    uint32_t n   = 0;
    uint32_t t0  = millis();
    while ((millis() - t0) < durationMs) {
        sum += readRawUncorrected();
        n++;
        delayMicroseconds(500);
    }
    if (n > 0) {
        _baseline = (int16_t)(sum / (int32_t)n);
    }

    resetFilters();
}

void easyECG::setBaseline(int16_t offsetCounts)
{
    _baseline = offsetCounts;
    resetFilters();
}

int16_t easyECG::getBaseline()
{
    return _baseline;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Optional / advanced low-level passthroughs (clear-then-set each field)
/////////////////////////////////////////////////////////////////////////////////////////

void easyECG::setDataRate(DataRate rate)
{
    _dr = (uint8_t)rate;
    applyConf();
}

void easyECG::setRange(FSRange range)
{
    _fsr = (uint8_t)range;
    applyConf();
    resetFilters();
}

void easyECG::setMode(Mode mode)
{
    _mode = (uint8_t)mode;
    applyConf();
}

void easyECG::setMux(Mux mux)
{
    _mux = (uint8_t)mux;
    applyConf();
}

int16_t easyECG::readSingleShot()
{
    _mode = MODE_SINGLE_SHOT;
    writeReg(EASYECG_CONF_REG, (uint16_t)(confWord() | 0x8000));

    static const uint16_t convMicros[] = {
        7813, 4000, 2041, 1087, 625, 417, 303,
    };
    uint16_t wait = (_dr <= DR_3300SPS) ? convMicros[_dr] : 7813;
    delayMicroseconds(wait);
    delayMicroseconds(wait);

    return readRaw();
}
