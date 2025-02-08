# ECE 3849 Lab 3 Audio Waveform

## Overview

This project is part of the ECE 3849 course and involves working with audio waveforms, FFT (Fast Fourier Transform), and displaying results on an LCD screen. The project uses the TM4C1294XL microcontroller and the Crystalfontz128x128 display.

## Project Structure

## Key Files

- **main.c**: The main file that initializes the system and starts the tasks.
- **audio_waveform.c**: Contains functions related to audio waveform processing.
- **audio_waveform.h**: Header file for audio waveform processing.
- **buttons.c**: Handles button debouncing and ISR for button presses.
- **buttons.h**: Header file for button handling.
- **Crystalfontz128x128_ST7735.c**: Driver for the Crystalfontz 128x128 display.
- **Crystalfontz128x128_ST7735.h**: Header file for the Crystalfontz display driver.
- **kiss_fft.c**: Implementation of the KISS FFT algorithm.
- **kiss_fft.h**: Header file for the KISS FFT algorithm.
- **_kiss_fft_guts.h**: Internal definitions and macros for the KISS FFT algorithm.

## Dependencies

- **TI-RTOS**: Real-time operating system for the TM4C1294XL microcontroller.
- **KISS FFT**: A simple FFT library.

## Building the Project

To build the project, use the provided `makefile.defs` and your preferred build system. Ensure that all dependencies are correctly set up.

## Running the Project

1. Load the project onto the TM4C1294XL microcontroller.
2. The system will initialize and start the tasks.
3. The audio waveform will be processed and displayed on the Crystalfontz 128x128 display.

## License

This project is licensed under the terms of the MIT license.

## Authors

- Gene Bogdanov

## Acknowledgments

- ECE 3849 course instructors and TAs.
