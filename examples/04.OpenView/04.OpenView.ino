// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, ProtoCentral Electronics
//
// 04.OpenView
//
// Streams the easyECG into the OpenView desktop app.
//
// ---- Which board to select in OpenView ---------------------------------------
//
//     In OpenView:  Connect -> USB -> select "easyECG Breakout"
//
// This needs OpenView 3.4 or later. On earlier versions, the "ADS1292R Breakout" profile
// also reads this stream, with the labels wrong: its ECG is the filtered trace, its
// Respiration the unfiltered trace, its HR the heart rate and its RR the signal status.
//
// Channels, at 57600 baud and 125 Hz:
//
//     ECG (filtered)   <- notch + 40 Hz low-pass, counts
//     ECG (unfiltered) <- plain average of the ADC samples, so you can see exactly what
//                        the filtering removes. Hidden by default in OpenView.
//                        Set SECOND_CHANNEL_UNFILTERED to false to send 0.
//     Heart rate       <- bpm
//     Signal           <- 0 = OK, 1 = lead-off / mains pickup, 2 = amplifier saturated
//                        (see below)
//
// ---- Sampling and filtering ----------------------------------------------------
//
// The library samples the ADC on a fixed 1000 SPS clock, applies a 50 Hz notch and a
// 4th-order 40 Hz Butterworth low-pass, and decimates to 125 Hz.
//
// No digital high-pass is applied by default: the board's 0.5 Hz analog corner is
// already there, and a second pole distorts the ST segment. Set HIGH_PASS_HZ to 0.5
// if baseline wander bothers you more than ST shape.
//
// ---- Electrodes ------------------------------------------------------------------
//
// Chest placement gives a much larger R-wave than wrists, and less arm-muscle noise:
//   LA (sleeve) - below the LEFT collarbone
//   RA (ring)   - below the RIGHT collarbone
//   RL (tip)    - lower right abdomen, or over the hip bone
// Sit still, arms relaxed and supported. Fresh gel electrodes, skin wiped with alcohol.
//
// SAFETY: use a battery-powered host or an isolated Qwiic adapter (easyISO). On a
// mains-powered host the driven-right-leg amplifier saturates and there is no trace.
//
// Download OpenView: https://github.com/Protocentral/protocentral_openview

#include <Wire.h>
#include "ProtoCentral_easyECG.h"

easyECG ecg;

// ---- user settings ----------------------------------------------------------------
#define MAINS_HZ                    50.0f   // 50 or 60
#define LOW_PASS_HZ                 40.0f   // 0 disables the digital low-pass
#define HIGH_PASS_HZ                0.0f    // 0 = off (keeps ST shape); 0.5 for wander
#define SECOND_CHANNEL_UNFILTERED   true

// ProtoCentral OpenView packet format
#define CES_CMDIF_PKT_START_1  0x0A
#define CES_CMDIF_PKT_START_2  0xFA
#define CES_CMDIF_TYPE_DATA    0x02
#define CES_CMDIF_PKT_STOP     0x0B
#define DATA_LEN               0x08
#define ZERO                   0

const uint8_t dataPacketHeader[5] = {CES_CMDIF_PKT_START_1, CES_CMDIF_PKT_START_2,
                                     DATA_LEN, ZERO, CES_CMDIF_TYPE_DATA};
const uint8_t dataPacketFooter[2] = {ZERO, CES_CMDIF_PKT_STOP};

void sendDataPacket(int16_t ecgCh, int16_t respCh, int16_t hr, int16_t rr)
{
    uint8_t p[DATA_LEN];
    p[0] = (uint8_t)ecgCh;  p[1] = (uint8_t)(ecgCh >> 8);
    p[2] = (uint8_t)respCh; p[3] = (uint8_t)(respCh >> 8);
    p[4] = (uint8_t)hr;     p[5] = (uint8_t)(hr >> 8);
    p[6] = (uint8_t)rr;     p[7] = (uint8_t)(rr >> 8);

    Serial.write(dataPacketHeader, 5);
    Serial.write(p, DATA_LEN);
    Serial.write(dataPacketFooter, 2);
}

void setup()
{
    Serial.begin(57600);

    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 3000)) {
        delay(10);
    }

    if (!ecg.begin()) {
        // No text output - the host is expecting binary packets.
        while (1) {
            delay(500);
        }
    }

    Wire.setClock(400000);

    ecg.setSampling(EASYECG_DEFAULT_ADC_RATE, 125);
    ecg.setNotch(MAINS_HZ);
    ecg.setLowPass(LOW_PASS_HZ);
    ecg.setHighPass(HIGH_PASS_HZ);

    // Zero the amplifier offset over one second. Sit still.
    ecg.calibrateBaseline(1000);
}

void loop()
{
    if (!ecg.update()) {
        return;
    }

    int16_t status = 0;
    if (ecg.isSaturated()) {
        status = 2;
    } else if (ecg.isLeadOff()) {
        status = 1;
    }

    int16_t second = SECOND_CHANNEL_UNFILTERED ? ecg.ecgUnfiltered() : 0;

    sendDataPacket(ecg.ecgFiltered(), second, (int16_t)(ecg.heartRate() + 0.5f), status);
}
