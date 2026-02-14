# Gesture-Controlled Robotic Hand

A complete system that tracks your hand in real-time using a webcam and controls a 5-finger robotic hand with servo motors.

## Features

- **Real-time Hand Tracking**: MediaPipe + OpenCV for accurate finger detection
- **5-DOF Control**: Independent thumb, index, middle, ring, and pinky control
- **Smooth Servo Movement**: Gradual transitions prevent jerky motion
- **Visual Feedback**: On-screen finger state display (OPEN/CLOSED in green/red)
- **Calibration Mode**: Test each finger individually
- **Serial Communication**: Sends "T,I,M,R,P\n" format to Arduino

## Hardware Required

- Arduino (Uno/Nano/Mega)
- 5x Servo motors (SG90, MG90S, or similar)
- External 5V power supply (recommended for 5 servos)
- Robotic hand mechanism (3D printed or mechanical)
- Webcam (built-in or external)

## Wiring Diagram

```
Servo Pins:
  Thumb  → Pin 3
  Index  → Pin 5
  Middle → Pin 6
  Ring   → Pin 9
  Pinky  → Pin 10

Power:
  Servo VCC (red)  → External 5V
  Servo GND (brown)→ Common GND
  Arduino GND      → Common GND
```

## Installation

### Python Setup

```bash
pip install opencv-python mediapipe pyserial
```

### Arduino Setup

1. Install Arduino IDE
2. Upload `robotic_hand_arduino.ino`
3. Note the serial port (e.g., COM3, /dev/ttyACM0)

### Configuration

Update `SERIAL_PORT` in Python script:

```python
# Windows
SERIAL_PORT = 'COM3'

# Linux
SERIAL_PORT = '/dev/ttyACM0'

# Mac
SERIAL_PORT = '/dev/tty.usbmodem14101'
```

## Usage

1. **Upload Arduino code** to your board
2. **Run Python script**:
   ```bash
   python hand_tracker.py
   ```
3. **Show your hand** to the webcam
4. **Watch the robotic hand** mirror your movements
5. **Press 'q'** to quit

## How It Works

### Python Side

- Uses MediaPipe to detect 21 hand landmarks
- Determines finger open/closed state based on joint positions
- Sends comma-separated values via serial: `1,0,1,0,1\n` (1=open, 0=closed)

### Arduino Side

- Parses incoming serial data
- Updates target positions for each servo
- Smoothly moves servos toward targets at 5°/update
- Includes calibration mode for setup

## Calibration

Adjust angle limits for your mechanical hand:

```cpp
const int CLOSED_ANGLE[5] = {0, 0, 0, 0, 0};   // Fist position
const int OPEN_ANGLE[5] = {180, 180, 180, 180, 180}; // Open hand
```

Run calibration sequence:

```cpp
// Uncomment in setup()
calibrateServos();
```

## Customization

| Parameter                  | Location | Description           |
| -------------------------- | -------- | --------------------- |
| `MOVE_SPEED`               | Arduino  | Servo speed (1-10)    |
| `MOVE_INTERVAL`            | Arduino  | Update timing (ms)    |
| `min_detection_confidence` | Python   | Detection sensitivity |
| `finger_tips/pips`         | Python   | Landmark indices      |

## Troubleshooting

| Issue                | Solution                                     |
| -------------------- | -------------------------------------------- |
| No serial connection | Check port name; close other serial monitors |
| Servos not moving    | Verify power; check pin assignments          |
| Jerky movement       | Add capacitor across servo power             |
| Hand not detected    | Good lighting; hand visible to camera        |
| Wrong finger mapping | Adjust landmark indices in Python            |

## Applications

- Prosthetic hand prototypes
- Animatronics and robotics
- Remote manipulation systems
- Sign language interpretation
- Educational demonstrations

## License

MIT - Free for learning and modification
