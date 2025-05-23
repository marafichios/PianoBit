# PianoBit

# PianoBit Project

## Overview
PianoBit is a microcontroller-based system featuring a 4x4 button matrix, 16 LEDs controlled via two daisy-chained 74HC595 shift registers, an I2C LCD display, and a buzzer for audio feedback. It plays musical notes corresponding to pressed keys and displays the note on the LCD.

## Features
- Efficient button matrix scanning using interrupts and timer multiplexing.
- LED control using shift registers, minimizing pin usage.
- Custom I2C communication for driving the LCD without external libraries.
- Software debouncing for reliable button input.
- Audible feedback through buzzer tones for each key press.

## Hardware
- Arduino Uno
- 4x4 Button Matrix
- 2x 74HC595 Shift Registers
- 16 LEDs with current-limiting resistors
- I2C LCD Display (16x2)
- Passive Buzzer

## Program Flow

1. **Initialization (`setup()`):**  
   - Initialize serial communication for debugging.  
   - Setup I2C and initialize the LCD display.  
   - Configure button matrix pins (rows as OUTPUT, columns as INPUT_PULLUP).  
   - Setup buzzer pin as OUTPUT.  
   - Configure timer and pin change interrupts to detect button presses and handle row multiplexing.  
   - Initialize shift registers and turn off all LEDs.

2. **Main Loop (`loop()`):**  
   - Check if an interrupt flagged a keypad change (`keypadChanged`).  
     - If yes, scan the keypad immediately, reset the flag, and update the last scan time.  
   - If no interrupt recently, periodically scan the keypad based on a time interval.  

3. **Keypad Scanning (`scanKeypad()`):**  
   - Activate each row one by one (set LOW), others HIGH.  
   - Read columns to detect if a button is pressed.  
   - Implement software debounce to filter noise and confirm stable button presses.  
   - On confirmed press:  
     - Update the LCD with the note name.  
     - Start buzzer tone for the corresponding frequency.  
     - Light up the associated LED via shift registers.  
   - On button release:  
     - Stop buzzer tone.  
     - Turn off LEDs.  
     - Reset LCD message.

4. **LED Control (`updateLED()` and `clearAllLEDs()`):**  
   - Use shift registers to control 16 LEDs using 3 Arduino pins (Data, Clock, Latch).  
   - Update LED pattern to light up the LED corresponding to the pressed button.  
   - Clear all LEDs when no button is pressed.

5. **Interrupts:**  
   - Timer interrupt cycles through rows for multiplexing the button matrix.  
   - Pin change interrupts detect changes in column inputs and set the keypad changed flag.


## Usage
1. Connect hardware as per the pin assignments in the source code.
2. Upload the firmware to Arduino using the Arduino IDE.
3. Press keys on the matrix to play notes, see the note on the LCD, and watch the corresponding LED light up.



