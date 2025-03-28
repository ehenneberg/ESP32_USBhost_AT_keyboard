// Software for an adapter to connect an USB keyboard to an IBM AT compatible motherboard
// select board as Waveshare ESP32-S3-Zero
// Note: only the PS/2 keyboard scan code set 2 is supported.!
// The keyboard USBhosting mostly originates from https://github.com/tanakamasayuki/EspUsbHost
// Hardware: ESP32-S3 zero, 2x BC547B, 7 resistors, 1 LED and USB-C to USB-A OTG connector
// Flashing the ESP32-S3 zero board goes via RX and TX pins using an ESP-01 programmer. 
// Key repeating takes place for: normal/shift letters and numbers, BkSP, Tab, Space, arrow keys
// The green LED lights up when a key at the USB keyboard is pressed 
// Further description at: https://larsenhenneberg.dk

#include "USBtoPS2_conversion_table.h" // USB keycode to PS/2 scan code set 2 conversion table
#include "EspUsbHost.h"                // ESP32 USBhost keyboard/mouse HID class definitions

// PS/2 clock and data In/Out pins as noted on the ESP32-S3 zero board
#define PS2clkOutPin  5   // pin to pull the PS/2 clock signal low via a BC547B transistor
#define PS2clkInPin   6   // pin to read the PS/2 clock signal via a voltage divider   
#define PS2dataOutPin 8   // pin to pull the PS/2 data signal low via a BC547B transistor
#define PS2dataInPin  7   // pin to read the PS/2 data signal via a voltage divider   
#define greenLedPin  11   // pin to green LED via a 220 ohm resistor
#define intrTestPin  10   // pin for PS/2 clock edge ISR test

// level for clock and data output, inverted states due to the open collector
// NPN transistors used for pulling the PS/2 bus signals physically low
#define PS2busOutLOW 1
#define PS2busOutHIGH 0

// define PS/2 bus states
#define nClk_nData  0x00    // clk = 0, data = 0
#define Clk_nData   0x01    // clk = 1, data = 0
#define nClk_Data   0x02    // clk = 0, data = 1
#define Clk_Data    0x03    // clk = 1, data = 1

// PS2 commands/requests from motherboard BIOS and DOS
#define setStatusCmd    0xED  // set keyboard LED status
#define diagnosticCmd   0xEE  // diagnostic
#define set_getScancode 0xF0  // set/get scan code set 2 , only set 2 available !
#define readIdCmd       0xF2  // read 2 byte keyboard ID
#define repeatRateCmd   0xF3  // set keyboard repeat key speed
#define keybEnableCmd   0xF4  // keyboard enable
#define keybDisableCmd  0xF5  // keyboard enable
#define ackCmd          0xFA  // Acknowledge
#define resendCmd       0xFE  // resend
#define resetKeybCmd    0xFF  // reset keyboard

// return values as response on motherboard commands
#define ackResponse 0xFA  // acknowledge returned on PS2 commands, except resend
#define readIdByte1 0xAB  // readId response value #1
#define readIdByte2 0x83  // readId response value #2
#define selfTestOk  0xAA  // returned value after resetKeyboard
#define scanCodeVal 0x41  // Scan set 2 value response

// PS/2 bus and init timing values
#define PS2powerUpDelay 2000        // power up delay in msec.
#define repeatKeyInterval 270       // repeat key interval on the key pressed in msec.
#define makeBreakPS2delay 40        // delay between PS2 make and break repeat key in msec.
#define delayBeforeRxPS2cmd 640     // delay before clocking in a PS/2 command in usec.
#define ackResponseDelay 700        // ack send_response delay on a PS/2 command in usec.
#define keyboardResetDelay 400000   // emulate AT keyboard reset delay in usec.
#define PS2cmdResponseDelay 950     // response delay on PS/2 command in usec.
#define PS2scanCodeDelay 1500       // delay before sending PS/2 keyboard scan code in usec.

// PS/2 bit banging timing delays taking the ISR time consumption into account
#define clkBitTime 40               // PS/2 clock bit time => 2 bit clk period ~ 12.5 kHz
#define clkLowBitTime clkBitTime-10 // clock low bit time - ISR time consumption !
#define clkHighBitTime 40           // PS/2 clock bit high time 
#define clkHalfBitTime 20           // PS/2 half clock bit time in usec.

bool gotMakeKey = false;            // USB make key (pressed) indicator for repeat key
uint32_t keyPressedTime;            // repeat key timer

uint8_t PS2cmd, lastPS2cmd, repeatRate, keyboardStatus, setScanCodeVal, lastUSBkeycode;

volatile bool PS2txBits[11];        // PS/2 scan code frame bits transmitted to PC

hw_timer_t *clkLowCounter = NULL;   // timer instance to measure a clock period of time
bool PS2cmdReady = false;           // indicator for pending PS/2 command from motherboard

/* The PS/2 clock edge triggered ISR, invoked on falling or rising clock edges:
1. the motherboard has a pending command => the clock signal is low more than 100 usec
2. issued clock signal to read the pending command from the motherboard
3. issued clock signal to send a PS/2 keyboard scan code to the motherboard
The clock low period is measured by resetting a 1 usec counter on the falling edge of
the clock signal and reading the counter on the rising edge of the clock signal. 
As the clock output is made with bit-banging delays, the clock low period will be
extended with the time it takes to complete the ISR twice, on falling and rising edge.
With an ESP32-S3 at 80 Mhz, completing the ISR twice takes approx 16 usec.
This value must be subtracted when bit-banging the clock low bit period.
NOTE: changing the ISR will change the ISR time consumption */

void IRAM_ATTR PS2clk_change_isr() {
  delayMicroseconds(5);                     // clock settling time
  if(digitalRead(PS2clkInPin) == LOW) {     // falling clock edge ?
    timerWrite(clkLowCounter, 0);           // set counter value = 0
    timerStart(clkLowCounter);              // start counter for clock low time
    digitalWrite(intrTestPin, LOW);         // falling clk edge => test pin low
  }
  else {                                    // rising clock edge here
    timerStop(clkLowCounter);                  // stop the clock low counter
    if ((timerRead(clkLowCounter) > 100) && (digitalRead(PS2dataInPin) == LOW)) {
      PS2cmdReady = true;                   // clock low time > 100 usec and data==0
    }                                       //  => PS/2 command pending
     digitalWrite(intrTestPin, HIGH);    // rising clk edge => test pin high
  }
}

// PS/2 scan code transmit with delay ahead of transmitting PS/2 scan code
void sendPS2scanbyte(uint8_t outbyte) {
  sendPS2byte(outbyte, PS2scanCodeDelay);
}

// convert the USB keycode to PS/2 scan codes via the USBtoPS2_conversion_table.h
// look up PS/2 scan code set 2 data and transmits the PS/2 scan codes. 
void sendPS2scanCode(uint8_t USBkeycode, uint8_t makeBreak, uint8_t modifier) {
  if (modifier == 0) {  // non modifier key
    if (makeBreak == 0) {                 // USB key pressed
      digitalWrite(greenLedPin, HIGH);    // break => green led off
      // repeat keys only for normal/shift letters and numbers, BkSP, Tab, Space, arrow keys !
      if (((USBkeycode >= 0x04) && (USBkeycode <= 0x27)) || ((USBkeycode >= 0x2A) && (USBkeycode <= 0x2C)) || ((USBkeycode >= 0x4F) && (USBkeycode <= 0x52))) {
        gotMakeKey = true;                  // indicate USB key pressed
        lastUSBkeycode = USBkeycode;
        keyPressedTime = millis();  // start timer for repeat key
      }
    } else {                              // USB key released
      digitalWrite(greenLedPin, LOW);     // make => green led on
      gotMakeKey = false;              // indicate USB key released
    }
    switch (USBkeycode) {
      case 0x48:              // PrtPause ?
        if (makeBreak == 1) {  // PrtPause and Make ? 
          for (int i=0; i < 8; i++) { sendPS2scanbyte(PrtPauseMake[i]); }
        }
        break;
      case 0x46:           // PrtScr ?
        if (makeBreak == 1) {
          for (int i=0; i < 4; i++) { sendPS2scanbyte(PrtScrMake[i]); }
        }
        else {
          for (int i=0; i < 6; i++) { sendPS2scanbyte(PrtScrBreak[i]); } 
        }
        break;
      default:
        if (makeBreak == 0) {         // USB key released
          sendPS2scanbyte(0xF0);      // break code
        }
        sendPS2scanbyte(USBtoPS2scanSet2[USBkeycode][0]);
        if (USBtoPS2scanSet2[USBkeycode][1] != 0) {
          sendPS2scanbyte(USBtoPS2scanSet2[USBkeycode][1]);
        }           
    } 
  }
  else {    // dispatch modifier keys
    uint8_t mod, modifierMask;
    for (int i=0; i < 7; i++) { // look through 7 modifier bits
      modifierMask = 0x01 << i;
      if ((modifier & modifierMask) != 0) { 
        mod = modifier & modifierMask; // mask out for one modifier bit
        switch (mod) {
          case 0x01:              // ctrlL ?
          if (makeBreak == 0) {
            sendPS2scanbyte(0xF0);  // break code
          }
          sendPS2scanbyte(0x14);
          break;
          case 0x02:              // shiftL ?
          if (makeBreak == 0) {
            sendPS2scanbyte(0xF0);  // break code
          }
          sendPS2scanbyte(0x12);
          break;
          case 0x04:              // altL ?
          if (makeBreak == 0) {
            sendPS2scanbyte(0xF0);  // break code
          }
          sendPS2scanbyte(0x11);
          break;
          case 0x08:              // window ?
          if (makeBreak == 0) {
            sendPS2scanbyte(0xF0);  // break code
          }
          sendPS2scanbyte(0xE0);
          sendPS2scanbyte(0x27);
          break;
          case 0x10:              // ctrlR ?
          if (makeBreak == 0) {
            sendPS2scanbyte(0xF0);  // break code
          }
          sendPS2scanbyte(0xE0);
          sendPS2scanbyte(0x14);
          break;
          case 0x20:              // shiftR ?
          if (makeBreak == 0) {
            sendPS2scanbyte(0xF0);  // break code
          }
          sendPS2scanbyte(0x59);
          break;
          case 0x40:              // altR (AltGr) ?
          if (makeBreak == 0) {
            sendPS2scanbyte(0xF0);  // break code
          }
          sendPS2scanbyte(0xE0);
          sendPS2scanbyte(0x11);
          break; 
          default:
          {}
        }
      }
    }
  }
}

// new USB keyboard data goes here, also see changes in EspUsbHost::_onReceive()
class MyEspUsbHost : public EspUsbHost {
// syntax for the usbHost->onKeyboardKey(...) function: makeBreak = 1 on keypressed and makeBreak = 0 on key released.
// usbHost->onKeyboardKey(usbHost->getKeycodeToAscii(report-keycode, shift), report-keycode, makeBreak, report-modifier);
  void onKeyboardKey(uint8_t ascii, uint8_t keycode, uint8_t makeBreak, uint8_t modifier) {
    sendPS2scanCode(keycode, makeBreak, modifier);
  };
};

MyEspUsbHost usbHost;

void setup() {
  delay(PS2powerUpDelay);                     // boot pwr supply settling delay
  pinMode(PS2clkOutPin, OUTPUT);              // set PS2 clk output pin as output
  digitalWrite(PS2clkOutPin, PS2busOutHIGH);  // PS2 clk high
  pinMode(PS2dataOutPin, OUTPUT);             // set PS2 data output pin as output
  digitalWrite(PS2dataOutPin, PS2busOutHIGH); // PS2 data high
  pinMode(PS2clkInPin, INPUT);                // set PS2 clk input pin as input
  pinMode(PS2dataInPin, INPUT);               // set PS2 data input pin as input
  pinMode(greenLedPin, OUTPUT);               // set green LED pin as output
  digitalWrite(greenLedPin, LOW);             // green led off
  pinMode(intrTestPin, OUTPUT);               // set ISR test pin as output
  digitalWrite(intrTestPin, LOW);             // set ISR test pin to low
  lastPS2cmd = 0x33;              // no PS2 cmd read yet, so set a non existing cmd
  clkLowCounter = timerBegin(1000000);        // 1000000 Hz timer/counter tick freq
  attachInterrupt(PS2clkInPin, PS2clk_change_isr, CHANGE);
  usbHost.begin();
  usbHost.setHIDLocal(HID_LOCAL_Danish);
}

// send a byte to the PS/2 bus, e.g. scan code or response to motherboard command
void sendPS2byte(uint8_t outbyte, uint32_t beforeDelay) {
  uint8_t outByte = outbyte;
  uint8_t trueBitCnt = 0;
  delayMicroseconds(beforeDelay);               // clk high delay for motherboard
  while (readPS2bus() != Clk_Data );            // wait clk=1 and data = 1
  digitalWrite(PS2dataOutPin, PS2busOutLOW);    // start bit, data low
  delayMicroseconds(clkHalfBitTime);            // 1/4 period start bit setup
  digitalWrite(PS2clkOutPin, PS2busOutLOW);     // neg. clk edge for start bit
  delayMicroseconds(clkLowBitTime);             // clk low period
  digitalWrite(PS2clkOutPin, PS2busOutHIGH);    // clk high
  for (int i=0; i<8; i++) {
    delayMicroseconds(clkHalfBitTime-7);        // 1/4 period data bit setup
    if ((outByte >> i) & 0x01) {                // bit==1 to be transmitted
      trueBitCnt++;                             // count no. of binary ones
      digitalWrite(PS2dataOutPin, PS2busOutHIGH); // set data bit==1
    }                                           
    else {
      digitalWrite(PS2dataOutPin, PS2busOutLOW);  // set data bit==0
    }
    delayMicroseconds(clkHalfBitTime+3);        // 1/4 period

    digitalWrite(PS2clkOutPin, PS2busOutLOW);   // neg. clk edge for data bit  
    delayMicroseconds(clkLowBitTime-4);         // clk low period
    digitalWrite(PS2clkOutPin, PS2busOutHIGH);  // clk high
  } 
  // send odd parity
  delayMicroseconds(clkHalfBitTime);            // 1/4 period data bit setup
  if ((trueBitCnt & 0x01 ) == 0)  { digitalWrite(PS2dataOutPin, PS2busOutHIGH); }
  else { digitalWrite(PS2dataOutPin, PS2busOutLOW); } // set odd parity
  delayMicroseconds(clkHalfBitTime);            // 1/4 period
  digitalWrite(PS2clkOutPin, PS2busOutLOW);     // neg. clk edge for parity bit  
  delayMicroseconds(clkLowBitTime);             // clk low period
  digitalWrite(PS2clkOutPin, PS2busOutHIGH);    // clk high  
  delayMicroseconds(clkHalfBitTime);            // 1/4 period stop bit setup
  digitalWrite(PS2dataOutPin, PS2busOutHIGH);   // set stop bit
  delayMicroseconds(clkHalfBitTime);            // 1/4 period
  digitalWrite(PS2clkOutPin, PS2busOutLOW);     // neg. clk edge for stop bit
  delayMicroseconds(clkLowBitTime);             // clk low period
  digitalWrite(PS2clkOutPin, PS2busOutHIGH);    // clk high
}

// read PS/2 command or data from the motherboard via the PS/2 bus
uint8_t readPS2cmd()
{
  uint8_t rxByte;
  bool PS2rxBits[11];
  delayMicroseconds(delayBeforeRxPS2cmd);   // delay for motherboard ahead of getting cmd
  digitalWrite(PS2clkOutPin, PS2busOutLOW); // PS2 clk low
  for (int i=0; i<9; i++) {
    delayMicroseconds(clkLowBitTime-6);
    digitalWrite(PS2clkOutPin, PS2busOutHIGH); // PS2 clk high
    PS2rxBits[i] = digitalRead(PS2dataInPin);
    delayMicroseconds(clkHighBitTime);
    digitalWrite(PS2clkOutPin, PS2busOutLOW); // PS2 clk low
  }
  delayMicroseconds(clkLowBitTime-6);
  digitalWrite(PS2clkOutPin, PS2busOutHIGH);  // PS2 clk high  
  delayMicroseconds(clkHalfBitTime);
  digitalWrite(PS2dataOutPin, PS2busOutLOW);  // PS2 data pin low , ACK start 
  delayMicroseconds(clkHalfBitTime);
  digitalWrite(PS2clkOutPin, PS2busOutLOW);   // PS2 clk low for sending ACK
  delayMicroseconds(clkLowBitTime-6);
  digitalWrite(PS2clkOutPin, PS2busOutHIGH);  // PS2 clk high
  delayMicroseconds(clkHalfBitTime);
  digitalWrite(PS2dataOutPin, PS2busOutHIGH); // PS2 data pin high , ACK end 
  delayMicroseconds(clkHalfBitTime);
  digitalWrite(PS2clkOutPin, PS2busOutLOW);   // PS2 clk low for sending ACK
  delayMicroseconds(clkLowBitTime-6);
  digitalWrite(PS2clkOutPin, PS2busOutHIGH);  // PS2 clk high
  rxByte = 0;
  for (int i=0; i<8; i++) {
    if (PS2rxBits[i]) {
      rxByte = rxByte | (0x01 << i);
    }
  }
  return rxByte;
}

// returns the PS/2 bus clk and data states as 8 bit: bit0 = clk , bit1 = data
uint8_t readPS2bus() {
  uint8_t busByte = 0;
  if (digitalRead(PS2dataInPin) == HIGH) { busByte = busByte | nClk_Data; }
  if (digitalRead(PS2clkInPin) == HIGH) { busByte = busByte | Clk_nData; }
 return busByte;
}

void parsePS2cmd() {
  PS2cmd = readPS2cmd();
  sendPS2byte(ackResponse, ackResponseDelay);   // response on commands, except resendCmd
  PS2cmdReady = false;
  delayMicroseconds(350);               // to cope with a motherboard processing time ( many clk changes )
  if (!PS2cmdReady) {                   // if no new PS2 cmd then
    delayMicroseconds(650);           
    if ( readPS2bus() == Clk_Data )  {   // clk = 1 and data = 1 => PS2bus tx ready for PS2 command response
      if (PS2cmd == resetKeybCmd) {
        sendPS2byte(selfTestOk, keyboardResetDelay);
      }
      if (PS2cmd == readIdCmd) {
        sendPS2byte(readIdByte1, PS2cmdResponseDelay);
        sendPS2byte(readIdByte2, PS2cmdResponseDelay);
      }
      if (PS2cmd == diagnosticCmd) {
        sendPS2byte(diagnosticCmd, PS2cmdResponseDelay);
      }
    }
    if (lastPS2cmd == repeatRateCmd) { // repeat rate value now ?
      repeatRate = PS2cmd;  
    }
    if (lastPS2cmd == setStatusCmd) { // set keyboard status now ?
      keyboardStatus = PS2cmd;  
    }
    if (lastPS2cmd == set_getScancode) { // set/get keyboard scan code set ?
      setScanCodeVal = PS2cmd;
      if (PS2cmd == 0x00) {   // response with actual scan code set ?
        sendPS2byte(scanCodeVal, PS2cmdResponseDelay);    // return scan code set 2 
      }
    }
  }
  lastPS2cmd = PS2cmd;
}

void loop() {
  if (PS2cmdReady) {  // pending PS/2 command from motherboard to be parsed ?
    parsePS2cmd();
  }
  // USB make key present longer the repeatKeyInterval ? then repeat/preempty current key
  if ((gotMakeKey == true) && ((millis() - keyPressedTime) > repeatKeyInterval)) {
    sendPS2scanbyte(0xF0);                                   // send a PS/2 break key
    sendPS2scanbyte(USBtoPS2scanSet2[lastUSBkeycode][0]);
    if (USBtoPS2scanSet2[lastUSBkeycode][1] != 0) {
      sendPS2scanbyte(USBtoPS2scanSet2[lastUSBkeycode][1]);
    } 
    delay(makeBreakPS2delay);                     // delay between PS/2 make and break
    sendPS2scanbyte(USBtoPS2scanSet2[lastUSBkeycode][0]);  // send a PS/2 make key
    if (USBtoPS2scanSet2[lastUSBkeycode][1] != 0) {
      sendPS2scanbyte(USBtoPS2scanSet2[lastUSBkeycode][1]);
    } 
    keyPressedTime = millis();                    // restart timer for repeat key
  }
  usbHost.task();
} 
