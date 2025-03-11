// Software for an adapter to connect an USB keyboard to an IBM AT compatible motherboard
// Note: only the PS/2 keyboard scan code set 2 is supported.!
// The keyboard USBhosting mostly originates from https://github.com/tanakamasayuki/EspUsbHost
// Hardware used: ESP32-S3 zero, 2x BC547B and 6 resistors, USB-C to USB-A OTG connector
// Flashing the ESP32-S3 zero board goes via RX and TX pins using an ESP-01 programmer. 
// Timing for in/out serial data is made with simple software delays to bitbang in/out data
// for the motherboard ( BIOS and OS ) and to avoid ESP32 interrupts
// Further description at: https://larsenhenneberg.dk

#include "USBtoPS2_conversion_table.h" // USB keycode to PS/2 scan code set 2 conversion table
#include "EspUsbHost.h"                // ESP32 USBhost keyboard/mouse HID class definitions

// PS/2 clock and data In/Out pin as noted on the ESP32-S3 zero board
#define PS2clkOutPin  5
#define PS2clkInPin   6
#define PS2dataOutPin 8
#define PS2dataInPin  7
#define greenLedPin 11

// level for clock and data output, inverted due to the open collector
// NPN transistors used for pulling the PS/2 bus physically low
#define PS2busOutLOW 1
#define PS2busOutHIGH 0

// define PS/2 bus states
#define nClk_nData  0x00    // clk = 0, data = 0
#define Clk_nData   0x01    // clk = 1, data = 0
#define nClk_Data   0x02    // clk = 0, data = 1
#define Clk_Data    0x03    // clk = 1, data = 1

// PS2 commands/requests from motherboard BIOS and DOS
// Note: these commands might differ from motherboard to motherboard,
// but any commands will return an ack ( 0xFA ) as the default response
// on command setStatusCmd 1 byte status is received and ack is returned
// on command readIdCmd, 2 keybord ID bytes are returned
#define setStatusCmd    0xED  // set keyboard LED status
#define diagnosticCmd   0xEE  // diagnostic
#define readIdCmd       0xF2  // read 2 byte keyboard ID
#define repeatRateCmd   0xF3  // set keyboard repeat key speed
#define keybEnableCmd   0xF4  // keyboard enable
#define keybDisableCmd  0xF5  // keyboard enable
#define ackCmd          0xFA  // Acknowledge
#define resendCmd       0xFE  // resend
#define resetKeybCmd    0xFF  // reset keyboard

// return values as response on motherboard commands
#define ack         0xFA  // acknowledge returned on PS2 commands, except resend
#define readIdByte1 0xAB  // readId return value #1
#define readIdByte2 0x83  // readId return value #2
#define selfTestOk  0xAA  // returned value after resetKeyboard

uint8_t PS2cmd, lastPS2cmd, repeatRate, keyboardStatus, setScanCodeVal;
uint32_t PS2powerUpDelay = 5000;  // power up delay in msec.
uint32_t initTimeOut = 30000;     // PS/2 init timeout in msec.
uint32_t previousTime;            // Previous time for timed loops
volatile bool PS2txBits[11];      // PS/2 scan code frame bits transmitted to PC

// PS/2 scan code transmit and send to the UART0 as well via Arduino Serial 
void sendPS2scanbyte(uint8_t outbyte) {
  sendPS2byte(outbyte);
  if (outbyte < 16) { Serial.print("0"); }
  Serial.print(outbyte, HEX);
  Serial.print(" ");
}

// convert the USB keycodes to PS/2 scan codes
// uses the USBtoPS2_conversion_table.h to find PS/2 scan code set 2 data
// and transmits PS/2 data. 
void sendPS2scanCode(uint8_t USBkeycode, uint8_t makeBreak, uint8_t modifier) {
  if (makeBreak == 0) {
    digitalWrite(greenLedPin, HIGH);   // break => green led off
  } else {
    digitalWrite(greenLedPin, LOW);  // make => green led on
  }
  if (modifier == 0) {  // non modifier key
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
        if (makeBreak == 0) {
          sendPS2scanbyte(0xF0);  // break code
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

// just add a preceeding 0 to show two digits for values < 16..
void printHex(uint8_t inbyte)
{
  Serial.print("0x");
  if (inbyte < 16) { Serial.print("0"); }
  Serial.print(inbyte, HEX);
}

// new USB keyboard data goes here, also see changes in EspUsbHost::_onReceive()
// keyboard codes and transmitted PS/2 scan codes are sent to the UART0 as well
// via Arduino Serial
class MyEspUsbHost : public EspUsbHost {
  void onKeyboardKey(uint8_t ascii, uint8_t keycode, uint8_t makeBreak, uint8_t modifier) {
    Serial.println();
    Serial.printf("keyc, makeBreak, mod: ");
    printHex(keycode);
    Serial.print(", ");
    printHex(makeBreak);
    Serial.print(", ");
    printHex(modifier);
    Serial.print(" . PS2scan: ");
    sendPS2scanCode(keycode, makeBreak, modifier);
    Serial.println();
  };
};

MyEspUsbHost usbHost;

void setup() {
  Serial.begin(115200);
  delay(PS2powerUpDelay);     // boot pwr supply settling delay
  pinMode(PS2clkOutPin, OUTPUT);    // set PS2 clk output pin as output
  digitalWrite(PS2clkOutPin, PS2busOutHIGH);  // PS2 clk high
  pinMode(PS2dataOutPin, OUTPUT);   // set PS2 data output pin as output
  digitalWrite(PS2dataOutPin, PS2busOutHIGH); // PS2 data high
  pinMode(PS2clkInPin, INPUT);      // set PS2 clk input pin as input
  pinMode(PS2dataInPin, INPUT);     // set PS2 data input pin as input
  pinMode(greenLedPin, OUTPUT);    // set PS2 clk output pin as output
  digitalWrite(greenLedPin, LOW);  // green led off
  lastPS2cmd = 0x33;      // no PS2 cmd read yet, so set a non existing cmd
  Serial.println("Enter " + String(initTimeOut/1000) + " sec. PS/2 boot sequence");
  initPS2();      // perform the start-up PS/2 initialization with motherboard 
  delay(500);
  usbHost.begin();
  usbHost.setHIDLocal(HID_LOCAL_Danish);
  previousTime = millis();
}

// send a byte to PS/2 bus ( scan code or response during initialization )
void sendPS2byte(uint8_t outbyte) {
  uint8_t outByte = outbyte;
  uint8_t trueBitCnt = 0;
  while (digitalRead(PS2clkInPin) == LOW) { };  // wait as long as clk is low
  delayMicroseconds(600);                      // clk high delay for motherboard
  digitalWrite(PS2dataOutPin, PS2busOutLOW);    // start bit, data low
  delayMicroseconds(20);                        // 1/4 period start bit setup
  digitalWrite(PS2clkOutPin, PS2busOutLOW);     // neg. clk edge for start bit
  delayMicroseconds(40);                        // clk low period
  digitalWrite(PS2clkOutPin, PS2busOutHIGH);    // clk high
  for (int i=0; i<8; i++) {
    delayMicroseconds(20);                      // 1/4 period data bit setup
    if ((outByte >> i) & 0x01) {
      trueBitCnt++;
      digitalWrite(PS2dataOutPin, PS2busOutHIGH);
    }                                           // set data bit
    else {
      digitalWrite(PS2dataOutPin, PS2busOutLOW);
    }
    delayMicroseconds(20);                      // 1/4 period
    digitalWrite(PS2clkOutPin, PS2busOutLOW);   // neg. clk edge for data bit  
    delayMicroseconds(40);                      // clk low period
    digitalWrite(PS2clkOutPin, PS2busOutHIGH);  // clk high
  } 
  // send odd parity
  delayMicroseconds(20);                        // 1/4 period data bit setup
  if ((trueBitCnt & 0x01 ) == 0)  { digitalWrite(PS2dataOutPin, PS2busOutHIGH); }
  else { digitalWrite(PS2dataOutPin, PS2busOutLOW); } // set odd parity
  delayMicroseconds(20);                        // 1/4 period
  digitalWrite(PS2clkOutPin, PS2busOutLOW);     // neg. clk edge for parity bit  
  delayMicroseconds(40);                        // clk low period
  digitalWrite(PS2clkOutPin, PS2busOutHIGH);    // clk high  
  delayMicroseconds(20);                        // 1/4 period stop bit setup
  digitalWrite(PS2dataOutPin, PS2busOutHIGH);   // set stop bit
  delayMicroseconds(20);                        // 1/4 period
  digitalWrite(PS2clkOutPin, PS2busOutLOW);     // neg. clk edge for stop bit
  delayMicroseconds(40);                        // clk low period
  digitalWrite(PS2clkOutPin, PS2busOutHIGH);    // clk high
}

// get a PS/2 command or data from the motherboard via the PS/2 bus
uint8_t readPS2cmd()
{
  uint8_t rxByte;
  bool PS2rxBits[11];
  delayMicroseconds(600);                  // delay for motherboard ahead of getting cmd
  digitalWrite(PS2clkOutPin, PS2busOutLOW); // PS2 clk low
  for (int i=0; i<9; i++) {
    delayMicroseconds(40);
    digitalWrite(PS2clkOutPin, PS2busOutHIGH); // PS2 clk high
    PS2rxBits[i] = digitalRead(PS2dataInPin);
    delayMicroseconds(40);
    digitalWrite(PS2clkOutPin, PS2busOutLOW); // PS2 clk low
  }
  delayMicroseconds(40);
  digitalWrite(PS2clkOutPin, PS2busOutHIGH); // PS2 clk high  
  delayMicroseconds(20);
  digitalWrite(PS2dataOutPin, PS2busOutLOW); // PS2 data pin low , ACK start 
  delayMicroseconds(20);
  digitalWrite(PS2clkOutPin, PS2busOutLOW); // PS2 clk low for sending ACK
  delayMicroseconds(40);
  digitalWrite(PS2clkOutPin, PS2busOutHIGH); // PS2 clk high
  delayMicroseconds(20);
  digitalWrite(PS2dataOutPin, PS2busOutHIGH); // PS2 data pin high , ACK end 
  delayMicroseconds(20);
  digitalWrite(PS2clkOutPin, PS2busOutLOW); // PS2 clk low for sending ACK
  delayMicroseconds(40);
  digitalWrite(PS2clkOutPin, PS2busOutHIGH); // PS2 clk high
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

// the tricky initialization part with motherboard using PS/2 protocol timing :)
void initPS2() {
  previousTime = millis();
  while ((millis() - previousTime) < initTimeOut) {
    while ((readPS2bus() != nClk_nData) && ((millis() - previousTime) < initTimeOut));  // wait for clk = 0 and data = 0
    while (( readPS2bus() == nClk_nData) && ((millis() - previousTime) < initTimeOut));  // wait while clk = 0 and data = 0
    if ( readPS2bus() == Clk_nData) {  // clk = 1 and data = 0 => PS2cmd ready
      PS2cmd = readPS2cmd();
      sendPS2byte(ack);   // response on commands, except resendCmd
      delayMicroseconds(350);           // to cope with a motherboard clk high 60 usec spurious
      if (readPS2bus() != nClk_nData) {
      delayMicroseconds(650);           // to cope with a motherboard clk high 60 usec spurious
      if ( readPS2bus() == Clk_Data )  {   // clk = 1 and data = 1 => PS2bus tx ready for PS2 command response
        if (PS2cmd == resetKeybCmd) {
          sendPS2byte(selfTestOk);
        }
        if (PS2cmd == readIdCmd) {
          sendPS2byte(readIdByte1);
          sendPS2byte(readIdByte2);
        }
        if (PS2cmd == diagnosticCmd) {
          sendPS2byte(diagnosticCmd);
        }
      }
      if (lastPS2cmd == repeatRateCmd) { // repeat rate value now ?
        repeatRate = PS2cmd;  
      }
      if (lastPS2cmd == setStatusCmd) { // set keyboard status now ?
        keyboardStatus = PS2cmd;  
        if (keyboardStatus == 0x02) {
          digitalWrite(greenLedPin, HIGH);  // green led on on NumLock LED on
        }
      }
      if (lastPS2cmd == setStatusCmd) { // read keyboard scan code set ?
        setScanCodeVal = PS2cmd;
        if (PS2cmd == 0x00) {
          sendPS2byte(0x41);          // return scan code set 2 
        }
      }
      }
      lastPS2cmd = PS2cmd;
    }
  }
  digitalWrite(greenLedPin, LOW);  // green led off
}

void loop() {
  usbHost.task();
}
