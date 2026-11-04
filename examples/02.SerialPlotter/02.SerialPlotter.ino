// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, ProtoCentral Electronics
//
// 02.SerialPlotter
//
// Plots the ECG in the Arduino IDE Serial Plotter (Tools -> Serial Plotter, 115200).
//
// Two traces, in uV at the electrodes:
//   Filtered   - 50 Hz notch + 40 Hz low-pass: what you normally want
//   Unfiltered - the plain ADC average, offset below so the two do not overlap
//
// Output is 250 Hz so the QRS keeps its shape. Set the plotter's window wide enough
// to show a few seconds, or the beats will scroll past too quickly to see.
//
// SAFETY: battery-powered host or isolated Qwiic adapter when the subject is a person.

#include <Wire.h>
#include "ProtoCentral_easyECG.h"

#define MAINS_HZ          50.0f    // 50 or 60
#define UNFILTERED_OFFSET 1500.0f  // uV, vertical spacing between the traces

easyECG ecg;

void setup()
{
    Serial.begin(115200);

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

    ecg.setSampling(EASYECG_DEFAULT_ADC_RATE, 250);
    ecg.setNotch(MAINS_HZ);
    ecg.calibrateBaseline(1000);
}

void loop()
{
    if (!ecg.update()) {
        return;
    }

    Serial.print("Filtered:");
    Serial.print(ecg.ecgMicrovolts(), 0);
    Serial.print(",Unfiltered:");
    Serial.println(ecg.ecgUnfilteredMicrovolts() - UNFILTERED_OFFSET, 0);
}
