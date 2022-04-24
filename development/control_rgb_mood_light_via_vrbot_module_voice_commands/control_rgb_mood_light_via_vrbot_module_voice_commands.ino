/*
 *  Control RGB Mood Light Via VRBot Module & Voice Commands.
 *
 *  Copyright (C) 2010 Efstathios Chatzikyriakidis (stathis.chatzikyriakidis@gmail.com)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

// include VRBot communication protocol commands.
#include "vrbot_protocol.h"

// the serial communication pin number for receiving data.
const uint8_t receivePin = 12;

// the serial communication pin number for transmitting data.
const uint8_t transmitPin = 13;

// the serial communication baud rate.
const int32_t baudRate = 9600;

// the period of a bit.
const int16_t bitPeriod = 1000000 / baudRate;

// the timeout for each communication (secs).
const uint8_t timeOut = 5;

// the pin number of the status led.
const uint8_t ledPin = 7;

// the delay time (ms) status led stay off.
const int32_t statusLedOffDelay = 2000;

const uint8_t redPin = 9;    // the red pin number of the rgb led.
const uint8_t greenPin = 10; // the green pin number of the rgb led.
const uint8_t bluePin = 11;  // the blue pin number of the rgb led.

uint8_t redVal = 0;   // the red value of the rgb led.
uint8_t greenVal = 0; // the green value of the rgb led.
uint8_t blueVal = 0;  // the blue value of the rgb led.

// delay time for color fading.
const int32_t colorFadingDelay = 30;

// startup point entry (runs once).
void
setup() {
  // set the led status pin as an output.
  pinMode(ledPin, OUTPUT);

  // set rgb led pins as an output.
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // PC UART pins initialization.
  pinMode(0, INPUT);
  pinMode(1, OUTPUT);

  // VRBot UART pins initialization.
  pinMode(receivePin, INPUT);
  pinMode(transmitPin, OUTPUT);
  
  // VRBot mode pins initialization.
  pinMode(2, INPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  // connect digital pin 2 to digital pin 3 to
  // enter normal mode - digital pin 2 is high.

  // connect digital pin 2 to digital pin 4 to
  // enter bridge mode - digital pin 2 is low.

  // set VRBot mode pins to HIGH/LOW according to datasheet.
  digitalWrite(3, HIGH);
  digitalWrite(4, LOW);

  // if digital pin 2 is LOW enter bridge mode.
  if (digitalRead(2) == LOW) {
    // bridge mode allow direct communication between
    // the VRBot module and the VRBot GUI application.
    while (true) {
      int16_t pc2vr = digitalRead(0);
      digitalWrite(transmitPin, pc2vr);
    
      int16_t vr2pc = digitalRead(receivePin);
      digitalWrite(1, vr2pc);
    }
  } 

  // if digital pin 2 is HIGH enter normal mode.

  // setup the serial line for communication.
  Serial.begin(baudRate);
  
  // delay some time before trying to setup VRBot.
  delay(200);

  // print verbose welcome messages.
  Serial.println("RGB Mood Light Controlled Via Voice Commands.");
  Serial.println("Enter Normal Mode.");

  // setup the VRBot.
  Serial.println("Try to setup VRBot.");
  VRBotSetup();

  // detect the VRBot.
  Serial.println("Try to detect VRBot.");
  if (!VRBotDetect())
    Serial.println("VRBot NA.");
  else {
    Serial.println("VRBot detected.");

    // set VRBot timeout (in secs).
    Serial.print("Setting timeout to: ");
    Serial.print(timeOut, DEC);
    Serial.println(" seconds.");
    VRBotSetTimeout(timeOut);

    // set VRBot language to English.
    Serial.println("Setting Language to: English.");
    VRBotSetLanguage(0);  
  } 
}

// loop the main sketch.
void
loop() {
  // handle speaker dependent recognition.
  SDRecognition();   
}

// update the values of the rgb led's pins.
void
update() {
  analogWrite(redPin, redVal);
  analogWrite(greenPin, greenVal);
  analogWrite(bluePin, blueVal);
}

// change the brightness of an rgb led color (light or dark).
void
color_morph(uint8_t * value, bool get_brighter) {
  // if the color has already the brightness
  // (light/dark) the user want just return.
  if (get_brighter && (*value) == 255) return;
  if (!get_brighter && (*value) == 0) return;

  // otherwise perform the appropriate brightness for the color.
  for (uint8_t i = 0; i < 255; i++) {
    if (get_brighter)
      (*value)++; // to light.
    else
      (*value)--; // to dark.

    // update the color of the rgb led.
    update();
    
    // delay some time for the rgb led to react.
    delay(colorFadingDelay);
  }
}

// speaker dependent recognition handler.
void
SDRecognition() {
  // speaker recognized command.
  int8_t cmd;

  // print verbose message.
  Serial.println("Say Trigger!");

  // status led on (light).
  statusLedOn(0);

  // start SD trigger message recognition and wait for trigger.
  VRBotRecognizeSD(0);

  // check recognition result.
  cmd = VRBotCheckResult();

  // status led off (dark).
  statusLedOff(statusLedOffDelay);

  // check for timeout result.
  if (cmd == -1) {
    Serial.println("Trigger Timeout.");
    return;
  }
  
  // check for error result.
  if (cmd == -2) {
    Serial.println("Trigger Error.");
    return;
  }
  
  // print verbose messages.
  Serial.println("Select Group 1.");
  Serial.println("Say Base Command!");
  
  // status led on (light).
  statusLedOn(0);

  // start SD recognition group 1 and wait for a command.
  VRBotRecognizeSD(1);

  // check recognition result.
  cmd = VRBotCheckResult();

  // status led off (dark).
  statusLedOff(statusLedOffDelay);

  // check the base command.
  switch (cmd) {
    case -2:
      Serial.println("Base Command Error.");
      break;

    case -1:
      Serial.println("Base Command Timeout.");
      break;

    case 0:
      // print verbose messages.
      Serial.println("Base Command: RED.");
      Serial.println("Say Sub Command!");

      // status led on (light).
      statusLedOn(0);

      // start SD recognition group 2 and wait for a command.
      VRBotRecognizeSD(2);

      // check recognition result.
      cmd = VRBotCheckResult();

      // status led off (dark).
      statusLedOff(statusLedOffDelay);

      // check the sub command.
      switch (cmd) {
        case -2:
          Serial.println("Sub Command Error.");
          break;

        case -1:
          Serial.println("Sub Command Timeout.");
          break;

        case 0:
          Serial.println("Sub Command: ON.");
          color_morph(&redVal, true);
          break;

        case 1:
          Serial.println("Sub Command: OFF.");
          color_morph(&redVal, false);
          break;

        default: // other sub command.
          Serial.println("Other Sub Command.");
          break;
      }
      break;

    case 1:
      // print verbose messages.
      Serial.println("Base Command: GREEN.");
      Serial.println("Say Sub Command!");

      // status led on (light).
      statusLedOn(0);

      // start SD recognition group 2 and wait for a command.
      VRBotRecognizeSD(2);

      // check recognition result.
      cmd = VRBotCheckResult();

      // status led off (dark).
      statusLedOff(statusLedOffDelay);

      // check the sub command.
      switch (cmd) {
        case -2:
          Serial.println("Sub Command Error.");
          break;

        case -1:
          Serial.println("Sub Command Timeout.");
          break;

        case 0:
          Serial.println("Sub Command: ON.");
          color_morph(&greenVal, true);
          break;

        case 1:
          Serial.println("Sub Command: OFF.");
          color_morph(&greenVal, false);
          break;

        default: // other sub command.
          Serial.println("Other Sub Command.");
          break;
      }
      break;

    case 2:
      // print verbose messages.
      Serial.println("Base Command: BLUE.");
      Serial.println("Say Sub Command!");

      // status led on (light).
      statusLedOn(0);

      // start SD recognition group 2 and wait for a command.
      VRBotRecognizeSD(2);

      // check recognition result.
      cmd = VRBotCheckResult();

      // status led off (dark).
      statusLedOff(statusLedOffDelay);

      // check the sub command.
      switch (cmd) {
        case -2:
          Serial.println("Sub Command Error.");
          break;

        case -1:
          Serial.println("Sub Command Timeout.");
          break;

        case 0:
          Serial.println("Sub Command: ON.");
          color_morph(&blueVal, true);
          break;

        case 1:
          Serial.println("Sub Command: OFF.");
          color_morph(&blueVal, false);
          break;

        default: // other sub command.
          Serial.println("Other Sub Command.");
          break;
      }
      break;

    case 3:
      // print verbose messages.
      Serial.println("Base Command: ALL.");
      Serial.println("Say Sub Command!");

      // status led on (light).
      statusLedOn(0);

      // start SD recognition group 2 and wait for a command.
      VRBotRecognizeSD(2);

      // check recognition result.
      cmd = VRBotCheckResult();

      // status led off (dark).
      statusLedOff(statusLedOffDelay);

      // check the sub command.
      switch (cmd) {
        case -2:
          Serial.println("Sub Command Error.");
          break;

        case -1:
          Serial.println("Sub Command Timeout.");
          break;

        case 0:
          Serial.println("Sub Command: ON.");
          color_morph(&redVal, true);
          color_morph(&greenVal, true);
          color_morph(&blueVal, true);
          break;

        case 1:
          Serial.println("Sub Command: OFF.");
          color_morph(&redVal, false);
          color_morph(&greenVal, false);
          color_morph(&blueVal, false);
          break;

        default: // other sub command.
          Serial.println("Other Sub Command.");
          break;
      }
      break;

    default: // other base command.
      Serial.println("Other Base Command.");
      break;
  }
}

// light the status led pin with delay time.
void
statusLedOn(int32_t time) {
  digitalWrite(ledPin, HIGH);
  delay(abs(time));
}

// dark the status led pin with delay time.
void
statusLedOff(int32_t time) {
  digitalWrite(ledPin, LOW);
  delay(abs(time));
}

// setup the VRBot.
void
VRBotSetup() {
  digitalWrite(transmitPin, HIGH);
  delayMicroseconds(bitPeriod); 
}

// read data from the VRBot.
uint8_t
VRBotRead() {
  // the data that will be read.
  uint8_t val = 0;

  // digitalRead delay is about 100 cycles.
  int16_t bitDelay = bitPeriod - clockCyclesToMicroseconds(100);
  
  // one byte of serial data (LSB first).
  // ...--\    /--\/--\/--\/--\/--\/--\/--\/--\/--...
  //   \--/\--/\--/\--/\--/\--/\--/\--/\--/
  //  start  0   1   2   3   4   5   6   7 stop

  while (digitalRead (receivePin));

  // confirm that this is a real start bit, not line noise.
  if (digitalRead(receivePin) == LOW) {
    // frame start indicated by a falling edge and low start bit.

    // jump to the middle of the low start bit.
    delayMicroseconds(bitDelay / 2 - clockCyclesToMicroseconds(50));
  
    // offset of the bit in the byte: from 0 (LSB) to 7 (MSB).
    for (uint8_t offset = 0; offset < 8; offset++) {
      // jump to middle of next bit.
      delayMicroseconds(bitDelay);

      // read bit.
      val |= digitalRead(receivePin) << offset;
    }
  
    delayMicroseconds(bitPeriod);

    // return the data.    
    return val;
  }
  
  return -1;
}

// write data to the VRBot.
void
VRBotWrite (uint8_t b) {
  // digitalWrite delay is about 50 cycles.
  int16_t bitDelay = bitPeriod - clockCyclesToMicroseconds(50);

  digitalWrite(transmitPin, LOW);
  delayMicroseconds(bitDelay);

  for (byte mask = 0x01; mask; mask <<= 1) {
    if (b & mask) // choose bit.
      digitalWrite(transmitPin, HIGH); // send 1.
    else
      digitalWrite(transmitPin, LOW); // send 0.

    delayMicroseconds(bitDelay);
  }

  digitalWrite(transmitPin, HIGH);
  delayMicroseconds(bitDelay);
}

// detect the VRBot.
uint8_t
VRBotDetect() {
  for (uint8_t i = 0; i < 5; ++i) {
    delay(100);    
    VRBotWrite(CMD_BREAK);        
    if (VRBotRead() == STS_SUCCESS)
      return 255;
  }

  return 0;
}

// set the language for the VRBot.
uint8_t
VRBotSetLanguage(uint8_t lang) {
  VRBotWrite(CMD_LANGUAGE);
  delay(5);
  VRBotWrite(ARG_ZERO + lang);

  if (VRBotRead() == STS_SUCCESS)
    return 255;

  return 0;
}

// try to recognize a speaker
// dependent group from VRBot.
void
VRBotRecognizeSD(uint8_t group) {
  VRBotWrite(CMD_RECOG_SD);
  delay(5);
  VRBotWrite(ARG_ZERO + group);
}

// set the timeout with the VRBot communication.
void
VRBotSetTimeout(uint8_t s) {
  VRBotWrite(CMD_TIMEOUT);
  delay(5);

  VRBotWrite(ARG_ZERO + s);
  delay(5);
}

// check the result from a possible VRBot recognition.
int8_t
VRBotCheckResult() {
  uint8_t rx = VRBotRead();

  if (rx == STS_SIMILAR || rx == STS_RESULT) {
    delay(5);
    VRBotWrite(ARG_ACK);

    // return command recognized.
    return (VRBotRead() - ARG_ZERO);
  }

  // timeout return code.
  if (rx == STS_TIMEOUT)
    return -1;
  
  // error return code.
  return -2;
}
