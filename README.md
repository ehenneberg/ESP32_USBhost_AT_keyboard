# ESP32_USBhost_AT_keyboard
ESP32-S3 as USB host to emulate an IBM AT compatible keyboard. The intention is to connect the emulator to an USB KVM switch.
The IBM AT keyboard emulation can use any USB keyboard as input and use the PS/2 keyboard protocol for PS/2 scan code set 2.
Hardware used is an ESP32-S3 zero board, 2 BC547B transistors, 7 resistors, 1 green LED and an OTG USB-C converter.
The code is made in Arduino IDE with an ESP 01 programmer to work with the ESP32-S3 UART0.

Hardware and tech. information at: https://larsenhenneberg.dk/
 
Notes:
1. The PS/2 keyboard emulation uses Scan code set 2 only
2. The code is using edge triggered interrupt and simple delays for the timing with the PS/2 bus.
3. Motherboards might have slightly different ways of PS/2 initializing in terms of dealing with the PS/2 bus timing.
   
![IBM_AT_USB_keyboard_HW](https://github.com/user-attachments/assets/f3f84d8d-aad0-4718-ab72-accd76cc90af)

Links:

https://wiki.osdev.org/USB_Human_Interface_Devices

https://wiki.osdev.org/PS/2_Keyboard

