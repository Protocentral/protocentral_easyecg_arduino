// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, ProtoCentral Electronics
//
// 03.HeartRate
//
// Prints the heart rate on every detected beat and blinks the built-in LED, and says
// so plainly when the signal cannot be trusted.
//
// The detector is a simplified Pan-Tompkins: it needs about 2 s to learn the signal
// level, then averages the last 8 R-R intervals. Expect a number after 3-5 beats.
//
// Status messages:
//   SATURATED - the amplifier is pinned at a rail. Usually a mains-powered host
//               (switch to battery, or use an isolated Qwiic adapter), or an
//               electrode that has come off.
//   LEAD OFF? - mains pickup is far above normal, which is what a loose or missing
//               electrode looks like. Provisional heuristic - see the README.
//
// SAFETY: battery-powered host or isolated Qwiic adapter when the subject is a person.

#include <Wire.h>
#include "ProtoCentral_easyECG.h"

#define MAINS_HZ 50.0f

#ifndef LED_BUILTIN
#define LED_BUILTIN 2   // not defined for every ESP32 board; GPIO2 is the usual on-board LED
#endif

easyECG ecg;

unsigned long ledOffAt     = 0;
unsigned long lastStatusAt = 0;

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 3000)) {
        delay(10);
    }

    if (!ecg.begin()) {
        Serial.println("easyECG not found - check wiring and I2C address (0x49).");
        while (1) {
            delay(1000);
        }
    }
    Wire.setClock(400000);

    ecg.setNotch(MAINS_HZ);
    ecg.calibrateBaseline(1000);

    Serial.println("Sit still - learning the signal for 2 s...");
}

void loop()
{
    unsigned long now = millis();

    if (ledOffAt && now >= ledOffAt) {
        digitalWrite(LED_BUILTIN, LOW);
        ledOffAt = 0;
    }

    if (!ecg.update()) {
        return;
    }

    if (ecg.beatDetected()) {
        digitalWrite(LED_BUILTIN, HIGH);
        ledOffAt = now + 60;

        Serial.print("Beat  RR ");
        Serial.print(ecg.lastRRMillis());
        Serial.print(" ms  HR ");
        float hr = ecg.heartRate();
        if (hr > 0.0f) {
            Serial.print(hr, 0);
            Serial.println(" bpm");
        } else {
            Serial.println("--");
        }
    }

    // Signal-quality status once a second.
    if (now - lastStatusAt >= 1000) {
        lastStatusAt = now;
        if (ecg.isSaturated()) {
            Serial.println("SATURATED - battery host or isolated adapter? electrode off?");
        } else if (ecg.isLeadOff()) {
            Serial.print("LEAD OFF? mains pickup ");
            Serial.print(ecg.mainsMicrovolts(), 0);
            Serial.println(" uV RMS");
        }
    }
}
