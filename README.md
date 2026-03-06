# Landslide Detection and Early Warning System using IoT and Computer Vision

A multi-modal landslide early warning system combining IoT environmental sensors with optical flow-based computer vision, deployed on edge hardware with LoRa wireless communication.

## Overview

The system addresses the high false alarm rate of sensor-only approaches by requiring visual confirmation of terrain displacement before triggering a RED zone alert. Three distributed nodes communicate over LoRa (433 MHz):

```
[Sensor Node]        [CV Node]           [Central Node]
Arduino Uno      +   Raspberry Pi 5   →  Raspberry Pi 5
Vibration            PiCamera IR          LoRa Receiver
Rain                 Optical Flow         Risk Fusion
Soil Moisture        LoRa TX (C0/C1)     Buzzer Alert
Tilt
LoRa TX (S0–S4)
```

**Risk classification:**
| Zone   | Condition                          | Buzzer |
|--------|------------------------------------|--------|
| GREEN  | CV=0, alertCount < 3               | OFF    |
| YELLOW | CV=0, alertCount ≥ 3              | OFF    |
| RED    | CV=1 (terrain displacement confirmed) | ON  |

## Hardware

| Node | Components |
|------|-----------|
| Sensor Node | Arduino Uno, SW-420 vibration, SW-520D rain, capacitive soil moisture, SW-200D tilt, RA-02 LoRa (SX1278, 433 MHz) |
| CV Node | Raspberry Pi 5 (8GB), 5MP IR Night Vision camera, RA-02 LoRa |
| Central Node | Raspberry Pi 5 (8GB), active buzzer, RA-02 LoRa |

## Repository Structure

```
├── arduino/
│   └── monitor_sensors.ino     # Sensor node — reads sensors, transmits S0–S4
├── cv_node/
│   ├── setup_rpicam.py         # One-time ROI setup, saves picamroi.json
│   ├── monitor_rpicam.py       # CV monitor — optical flow, LoRa TX (display mode)
│   ├── monitor_rpicam_lora.py  # CV monitor — optical flow, LoRa TX (production)
│   └── monitor_rpicam_log.py   # CV monitor — with JSON session logging
├── central_node/
│   └── receiver.py             # Receives C/S packets, risk fusion, buzzer control
├── picamroi.template.json      # Template for ROI configuration
└── requirements.txt
```

## Setup

### 1. Install dependencies (CV Node and Central Node)

```bash
pip install -r requirements.txt
```

### 2. Configure ROI (CV Node)

Run once to define the Region of Interest on the monitored hillslope:

```bash
python cv_node/setup_rpicam.py
```

Click to place 4 corners around the monitored area, press `s` to save. Creates `picamroi.json`.

### 3. Flash Arduino (Sensor Node)

Open `arduino/monitor_sensors.ino` in Arduino IDE and upload to Arduino Uno. Requires the [Arduino-LoRa](https://github.com/sandeepmistry/arduino-LoRa) library.

### 4. Run the system

On **CV Node** (Raspberry Pi 5 with camera):
```bash
python cv_node/monitor_rpicam_lora.py
```

On **Central Node** (Raspberry Pi 5 with buzzer):
```bash
python central_node/receiver.py
```

## CV Detection Algorithm

Farnebäck dense optical flow is computed on consecutive infrared camera frames within a user-defined ROI. A pixel is classified as exhibiting landslide-indicative motion if:

- Flow magnitude > 1.5 pixels/frame
- Downward alignment coefficient > 0.9
- At least 200 such pixels present in ROI

A detection flag (`CV=1`) is issued only when these conditions persist for ≥ 3 consecutive frames. This temporal persistence filter reduces false positives from transient motion (wind, vegetation).

## Session Logging

`monitor_rpicam_log.py` saves a timestamped JSON file on exit containing per-frame metrics and summary statistics:

```bash
python cv_node/monitor_rpicam_log.py
# Saves: session_YYYYMMDD_HHMMSS.json
```

## LoRa Configuration

| Parameter | Value |
|-----------|-------|
| Frequency | 433 MHz |
| Spreading Factor | SF7 |
| Bandwidth | 125 kHz |
| Coding Rate | 4/5 |
| Sync Word | 0x12 |
| TX Power | +18 dBm |

## Requirements

- Raspberry Pi 5 (8GB recommended)
- Python 3.10+
- PiCamera2
- OpenCV 4.x
- Arduino Uno with LoRa RA-02 (SX1278)

## License

MIT License
