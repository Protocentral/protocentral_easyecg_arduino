// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, ProtoCentral Electronics
//
// 01.BasicReading
//
// The "hello world": initialise the board and print the filtered ECG in ADC counts
// and in microvolts at the electrodes, 125 times a second.
//
// Hardware connections (Qwiic / STEMMA QT or breakout header):
//   VCC -> 3.3V        SDA -> I2C Data
//   GND -> Ground      SCL -> I2C Clock
//
// Electrode placement (chest, Lead I):
//   LA (sleeve) - below the LEFT collarbone
//   RA (ring)   - below the RIGHT collarbone
//   RL (tip)    - lower right abdomen, or over the hip bone
//
// SAFETY: easyECG has no patient isolation and is not a medical device. When the
// subject is a person, use a battery-powered host or an isolated Qwiic adapter.

#include <Wire.h>
#include "ProtoCentral_easyECG.h"

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

    Serial.println("ProtoCentral easyECG ready. Sit still...");
    ecg.calibrateBaseline(1000);

    Serial.println("ECG(counts)\tECG(uV)");
}

void loop()
{
    // update() takes ADC samples on a fixed 1000 SPS clock and returns true each time
    // a filtered output sample is ready (125 per second by default).
    if (ecg.update()) {
        Serial.print(ecg.ecgFiltered());
        Serial.print("\t");
        Serial.println(ecg.ecgMicrovolts(), 1);
    }
}
