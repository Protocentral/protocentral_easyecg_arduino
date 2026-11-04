# Changelog

All notable changes to this library are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/).

## [1.0.0] - 2026-11-04

First public release.

### Added
- Fixed-rate ADC sampling at 1000 SPS (500 SPS on 8-bit AVR), with decimation to 125 or 250 Hz.
- 50/60 Hz notch, 4th-order 40 Hz Butterworth low-pass and an optional high-pass.
- Heart rate from a simplified Pan-Tompkins R-peak detector.
- Saturation, mains-pickup and lead-off flags.
- Readings in microvolts at the electrodes.
- Examples 01 to 05, including OpenView streaming and diagnostics.
