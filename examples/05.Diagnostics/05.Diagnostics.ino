// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, ProtoCentral Electronics
//
// 05.Diagnostics
//
// Run this when the trace looks wrong and you want numbers instead of guesses.
//
//   1. Is the ADC on the bus, and at which address?
//   2. Did the configuration take? (CONFIG read back - expect 0x04C3)
//   3. Is the conversion register changing at all?
//   4. Every 2 s: noise and mains figures for the live signal.
//
// Phase 4 prints, per 2 s window, in uV at the electrodes:
//
//   filt_rms   RMS of the filtered output   - what OpenView's ECG channel shows
//   raw_rms    RMS of the unfiltered output
//   mains      mains pickup the notch is removing
//   pp         peak-to-peak of the filtered output (~R-wave amplitude with a heart)
//   hr         heart rate, bpm
//   sat / lead amplifier saturated / lead-off heuristic
//   missed     samples the loop could not take on time (should stay 0)
//
// The two tests that separate board noise from body noise:
//
//   a) SHORT LA, RA and RL together at the jack (or clip all three leads to one
//      electrode). filt_rms is now the board's own noise floor - expect a few uV.
//   b) Electrodes on the body, subject still. If filt_rms between beats is much
//      larger than (a) and mains is small, the extra noise is muscle or electrode
//      contact - fix placement and skin prep; filtering will not remove it.
//
// Open the Serial Monitor at 115200.
//
// SAFETY: battery-powered host or isolated Qwiic adapter when the subject is a person.

#include <Wire.h>
#include "ProtoCentral_easyECG.h"

#define MAINS_HZ     50.0f

// One decimal place, right-aligned. dtostrf() isn't declared on every core, and printf
// %f isn't available on AVR.
static void fmt1(float v, int width, char *out, size_t len)
{
    long tenths = lround(v * 10.0f);
    long mag = labs(tenths);
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%s%ld.%ld", tenths < 0 ? "-" : "", mag / 10, mag % 10);
    snprintf(out, len, "%*s", width, tmp);
}

easyECG ecg;
uint8_t foundAddr = 0;

void printHex16(uint16_t v)
{
    Serial.print(F("0x"));
    for (int8_t i = 12; i >= 0; i -= 4) {
        Serial.print((v >> i) & 0x0F, HEX);
    }
}

void setup()
{
    Serial.begin(115200);
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 3000)) {
        delay(10);
    }
    Wire.begin();

    Serial.println();
    Serial.println(F("=== ProtoCentral easyECG diagnostics ==="));

    // 1. I2C scan
    Serial.println(F("--- 1. I2C scan"));
    for (uint8_t a = 0x08; a < 0x78; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("   device at 0x"));
            Serial.print(a, HEX);
            if (a == EASYECG_I2C_ADDR_DEFAULT || a == EASYECG_I2C_ADDR_ALT) {
                Serial.print(F("  <- easyECG"));
                if (!foundAddr) foundAddr = a;
            }
            Serial.println();
        }
    }
    if (!foundAddr) {
        Serial.println(F("   easyECG not found at 0x49/0x48. Check wiring and power."));
        while (1) delay(1000);
    }

    ecg = easyECG(foundAddr);
    ecg.begin();
    Wire.setClock(400000);
    ecg.setSampling(EASYECG_DEFAULT_ADC_RATE, 250);
    ecg.setNotch(MAINS_HZ);

    // 2. CONFIG read-back. Bit 15 reads 0 while a conversion is in progress.
    Serial.println(F("--- 2. CONFIG read-back"));
    uint16_t conf = ecg.readConfig();
    Serial.print(F("   read "));
    printHex16(conf);
    Serial.print(F(", expected 0x04C3 (+/-2.048 V, continuous, 3300 SPS)  "));
    Serial.println(((conf & 0x7FFF) == 0x04C3) ? F("OK") : F("MISMATCH"));

    // 3. Is the conversion register changing?
    Serial.println(F("--- 3. Conversion register activity"));
    int16_t mn = 32767, mx = -32768;
    long    sum = 0;
    for (uint16_t i = 0; i < 500; i++) {
        int16_t v = ecg.readRawUncorrected();
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
        delayMicroseconds(1000);
    }
    Serial.print(F("   500 samples: min "));
    Serial.print(mn);
    Serial.print(F("  max "));
    Serial.print(mx);
    Serial.print(F("  mean "));
    Serial.println(sum / 500);
    if (mn == mx) {
        Serial.println(F("   STUCK: the ADC returns a constant. Check power and the CONFIG word."));
    } else if (mx >= EASYECG_SATURATION_COUNTS || mn <= -EASYECG_SATURATION_COUNTS) {
        Serial.println(F("   Near a rail: electrode off, or mains-powered host without isolation."));
    } else {
        Serial.println(F("   OK - converting."));
    }

    ecg.calibrateBaseline(1000);
    Serial.print(F("   baseline "));
    Serial.print(ecg.getBaseline());
    Serial.println(F(" counts"));

    Serial.println(F("--- 4. Live figures, uV at the electrodes, every 2 s"));
    Serial.println(F("   filt_rms  raw_rms   mains     pp        hr     sat lead missed"));
}

// Accumulators for one reporting window.
float    sumF = 0, sumF2 = 0, sumR = 0, sumR2 = 0;
float    minF = 1e9, maxF = -1e9;
uint16_t nWin = 0;
uint8_t  skipWindows = 1;   // let the filters settle first

void loop()
{
    if (!ecg.update()) {
        return;
    }

    float f = ecg.ecgMicrovolts();
    float r = ecg.ecgUnfilteredMicrovolts();
    sumF += f;  sumF2 += f * f;
    sumR += r;  sumR2 += r * r;
    if (f < minF) minF = f;
    if (f > maxF) maxF = f;
    nWin++;

    if (nWin < 2 * ecg.outputRateHz()) {
        return;
    }

    if (skipWindows > 0) {
        skipWindows--;
    } else {
        float mf = sumF / nWin, mr = sumR / nWin;
        float rmsF = sqrtf(max(0.0f, sumF2 / nWin - mf * mf));
        float rmsR = sqrtf(max(0.0f, sumR2 / nWin - mr * mr));

        char line[96];
        char a[12], b[12], c[12], d[12], e[12];
        fmt1(rmsF, 8, a, sizeof(a));
        fmt1(rmsR, 8, b, sizeof(b));
        fmt1(ecg.mainsMicrovolts(), 8, c, sizeof(c));
        fmt1(maxF - minF, 8, d, sizeof(d));
        fmt1(ecg.heartRate(), 6, e, sizeof(e));
        snprintf(line, sizeof(line), "   %s  %s  %s  %s  %s   %s  %s  %lu",
                 a, b, c, d, e,
                 ecg.isSaturated() ? "Y" : "-",
                 ecg.isLeadOff() ? "Y" : "-",
                 (unsigned long)ecg.missedSamples());
        Serial.println(line);
    }

    sumF = sumF2 = sumR = sumR2 = 0;
    minF = 1e9;
    maxF = -1e9;
    nWin = 0;
}
