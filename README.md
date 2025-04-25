# ESP32 Automated Guitar Tuner

An automated guitar tuning system built on the ESP32 platform that uses audio frequency detection and motor control to automatically tune guitar strings.

## Features

- Real-time frequency detection using FFT analysis
- Automatic string identification and tuning
- OLED display interface showing tuning status
- Push-button operation
- Motor-driven tuning mechanism
- Support for standard guitar tuning (E2, A2, D3, G3, B3, E4)

## Hardware Requirements

- ESP32 Heltec Board
- DC Motor with Driver
- Microphone Module (ADC input)
- Push Button
- Power Supply

### Pin Configuration

#### OLED Display (I2C)
- SDA: GPIO 17
- SCL: GPIO 18
- RST: GPIO 21

#### Motor Driver
- IN1: GPIO 5
- IN2: GPIO 4

#### Input
- Button: GPIO 19
- Microphone: ADC1 Channel 6

## Software Dependencies

- ESP-IDF Framework
- LVGL Library for display
- kissfft Library for FFT computation

## Key Parameters

### Audio Processing
- Sampling Rate: 4096 Hz
- Sample Size: 4096 samples
- Moving Average Filter Size: 10 samples

### Tuning Parameters
- Tuning Tolerance: within 1 Hz
- Delay Scale: 60 ms (for motor control timing)
- Frequency Samples: 3 (for averaging)

### Supported Notes and Frequencies
```c
E2: 185.05 Hz
A2: 124.10 Hz
D3: 164.90 Hz
G3: 220.50 Hz
B3: 278.90 Hz
E4: 64.80 Hz
```

## How It Works

1. **Initialization**
   - Configures GPIO pins for button input and motor control
   - Sets up I2C communication for OLED display
   - Initializes ADC for microphone input
   - Configures LVGL for display interface

2. **Operation Flow**
   - User presses button to start tuning process
   - System samples audio from microphone
   - FFT analysis determines dominant frequency
   - Closest musical note is identified
   - Motor adjusts string tension based on frequency difference
   - Display shows real-time tuning status

3. **Display Interface**
   - Shows "Tuning..." during adjustment
   - Displays string being tuned
   - Shows frequency difference and polarity
   - Indicates when tuning is complete

## Usage

1. Power on the device
2. Pluck the guitar string you want to tune
3. Press the button to start tuning process
4. Wait for the motor to adjust the string tension
5. When "Tuning complete" is displayed, the string is properly tuned
6. Repeat for other strings

## Technical Details

### Signal Processing
- Uses moving average filter for noise reduction
- Implements FFT for frequency analysis
- Quadratic interpolation for improved frequency accuracy

### Motor Control
- Bidirectional control for tightening/loosening
- Proportional control based on frequency difference
- Automatic stop when within tuning tolerance

## Limitations

- Works best in quiet environments
- Requires proper microphone positioning
- Motor control timing may need adjustment based on specific hardware
- Limited to standard guitar tuning frequencies

## Future Improvements

- Add support for alternative tunings
- Implement PWM control for smoother motor operation
- Switch to servo motor for more precise tuning
- Support for different string instruments