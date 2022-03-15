# Running
This project is designed to run a DIY Audio Synthesizer using an STM32 board.

In order to run this project, you must go into the files for the arduino compiler and replace the `1.9.0` folder in `..\Arduino15\packages\STM32\hardware\stm32\1.9.0` with the `1.9.0` folder in this repo.

Then move into the `main` directory and compile the `main.ino` file.

# Project structure
## Folder `1.9.0`
This is a clone of the arduino drivers repo. Only a few modifications have been made to it to enable advanced functionality (enabling the Direct Memory Access - DMA)

## Folder `main`
Contains the main source code to run