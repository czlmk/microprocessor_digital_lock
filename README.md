# Digital lock
- Project created using STM32CubeMX
- A digital lock ran on STM32L4+ Series STM32L4S5VIT6 microcontroller based on the Arm® Cortex®-M4 core.
- Capable of recording and storing messages in morse code, can be displayed through anything connectable through UART.
- Has power management system which forces the board to go into sleep state whenever the board is locked or after 30 seconds without input.

# Usage
- First wake up device by pressing a button on board.
- Unlock through raising humidity and a sound will be played through speaker once unlocked/
- Pressing the button again will allow the board to go into either read or write mode.
  - In read mode previously recorded messages will be shown seperated by rows.
  - in record mode, the user can record in morse code with a time limit of 30 seconds.
    - single press records a dot
    - double press records a dash
    - triple press records a pause
    - recording process can be ended with 4 presses (goes back to where user choose which mode to enter)
  - All messages displayed through UART connection
- Lock by shaking the board (triggered through accelerometer) which allows the board to go into sleep state
   
