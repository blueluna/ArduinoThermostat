/*
  Read the temperature from a LM35 using analog pin A0.
  Control a relay on D8 with a button on A5.

  Copyright 2012 Erik Svensson <erik.public@gmail.com>

  Tested with Arduino 1.0.2
*/

#include <OneWire.h>
#include <ErikLib.h>
#include <PinChangeInt.h>
#include <Wire.h>

#define DISABLE_PCINT_MULTI_SERVICE

const uint8_t relay1Pin = 7;
const uint8_t relay2Pin = 8;
const uint8_t ledPin = 13;
const uint8_t btnPin = A5;
const uint8_t owPin = 5;
const uint8_t rotA = 10;
const uint8_t rotB = 9;
const uint8_t rotRed = 11;
const uint8_t rotGreen = 3;
const uint8_t rotBtn = 4;

const uint32_t relaySwitchDelay = 500; // milliseconds
uint32_t relaySwitchTime = 0; // milliseconds
uint8_t relayActive = 0; // the active relay (1 or 2)
uint8_t relayToActivate = 0; // the relay to activate (1 or 2)

OneWire ow(owPin); // The 1-wire bus
uint8_t owAddressArrayCapacity = 0; // capacity for array of 1-wire device addresses
uint8_t *owAddressArray = 0; // array of 1-wire device addresses
uint8_t owAddressArrayCount = 0; // number of devices in array of 1-wire device addresses

uint8_t lastEncoded = 0;
uint8_t encoderValue = 0;


enum {
  TM_NORMAL = 0,
  TM_LOW
};
uint8_t thermostatMode = TM_NORMAL;

NMEASentence sentence(128);
Button btnBoard(btnPin, 50);
Button btnRot(rotBtn, 50);

enum {
  OW_RESET = 0,
  OW_CONFIGURED,
  OW_IDLE,
  OW_WAIT_FOR_CONVERSION,
  OW_READ_TEMPERATURE = 0x80
};

uint8_t owState = OW_RESET; // current state of the 1-wire state machine
uint32_t owStateTime = 0; // time for the last state change

void setup()
{
  uint32_t t = millis();

  pinMode(relay1Pin, OUTPUT); 
  digitalWrite(relay1Pin, LOW);
  pinMode(relay2Pin, OUTPUT); 
  digitalWrite(relay2Pin, HIGH);
  relaySwitchTime = t;
  relayActive = 2;
  relayToActivate = 2;

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  pinMode(btnPin, INPUT_PULLUP);

  pinMode(owPin, OUTPUT); 
  digitalWrite(owPin, HIGH);

  pinMode(rotA, INPUT); 
  digitalWrite(rotA, HIGH);
  pinMode(rotB, INPUT); 
  digitalWrite(rotB, HIGH);
  pinMode(rotBtn, INPUT); 
  digitalWrite(rotBtn, HIGH);
  pinMode(rotRed, OUTPUT); 
  digitalWrite(rotRed, HIGH);
  pinMode(rotGreen, OUTPUT); 
  digitalWrite(rotGreen, LOW);
  delay(100);
  digitalWrite(rotRed, LOW);
  digitalWrite(rotGreen, HIGH);
  delay(100);
  digitalWrite(rotRed, HIGH);
  digitalWrite(rotGreen, HIGH);
  delay(100);
  digitalWrite(rotRed, LOW);
  digitalWrite(rotGreen, LOW);

  Serial.begin(115200);

  Wire.begin();

  owState = OW_RESET;
  owHandler(t);

  PCintPort::attachInterrupt(rotA, &updateEncoder, CHANGE);
  PCintPort::attachInterrupt(rotB, &updateEncoder, CHANGE);
}

void loop()
{
  uint8_t state;
  uint32_t t = millis();
  // Use the button to control the relay
  state = btnBoard.Check(t);
  if (state == Button::Falling) {
    switchRelays(t);
  }
  state = btnRot.Check(t);
  if (state == Button::Falling) {
    thermostatMode = (thermostatMode == TM_NORMAL) ? TM_LOW : TM_NORMAL;
    if (thermostatMode == TM_NORMAL) {
      digitalWrite(rotRed, HIGH);
      digitalWrite(rotGreen, LOW);
    }
    else {
      digitalWrite(rotGreen, HIGH);
      digitalWrite(rotRed, LOW);
    }
  }
  handleRelays(t);
  owHandler(t);

  analogWrite(ledPin, encoderValue);
}

void handleRelays(const uint32_t t)
{
  if (relayActive != relayToActivate) {
    if (t >= (relaySwitchTime + relaySwitchDelay)) {
      digitalWrite(relay1Pin, LOW);
      digitalWrite(relay2Pin, LOW);
      if (relayToActivate == 1) {
        digitalWrite(relay1Pin, HIGH);
      }
      else if (relayToActivate == 2) {
        digitalWrite(relay2Pin, HIGH);
      }
      relayActive = relayToActivate;
    }
  }
}

void switchRelays(const uint32_t t)
{
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, LOW);
  relayToActivate = relayActive == 2 ? 1 : 2;
  relaySwitchTime = t;
}

void owHandler(const uint32_t t)
{
  uint8_t state = owState;
  if ((state & OW_READ_TEMPERATURE) == OW_READ_TEMPERATURE) {
    state = OW_READ_TEMPERATURE;
  }
  switch (state) {
    case OW_RESET:
      owScanBus();
      owStateTime = t;
      break;
    case OW_CONFIGURED:
      owState = OW_IDLE;
      owStateTime = t;
      break;
    case OW_IDLE:
      if (t > (owStateTime + 5000)) {
        owStartConversion();
        owStateTime = millis();
      }
      break;
    case OW_WAIT_FOR_CONVERSION:
      if (t < (owStateTime + 1000)) {
        break;
      }
      owState = OW_READ_TEMPERATURE;
      // NO BREAK!
    case OW_READ_TEMPERATURE:
      owReadTemperature();
      owStateTime = t;
      break;
  }
}

void owScanBus()
{
  uint8_t status = ow.reset();
  if (status == 0) {
    return;
  }
  ow.reset_search();
  
  uint8_t address[8];
  uint8_t count = 0;
  uint8_t more = ow.search(address);
  while (more == 1) {
    count++;
    more = ow.search(address);
  }
  owAddressArrayCount = 0;
  if (count > 0) {
    ow.reset_search();
    if (owAddressArrayCapacity < count) {
      if (owAddressArrayCapacity > 0) {
        free(owAddressArray);
        owAddressArrayCapacity = 0;
      }
      owAddressArray = reinterpret_cast<uint8_t*>(malloc(count * 8));
      owAddressArrayCapacity = count;
    }

    uint8_t *ptr = owAddressArray;
    more = ow.search(ptr);
    for (uint8_t n = 0; n < count && more == 1; n++) {
      sentence.clear();
      sentence.print("SCN,");
      for (int i = 0; i < 8; i++) {
        ErikLib::HexPrint(sentence, *(ptr + i));
      }
      Serial.print(sentence);
      ptr += 8;
      owAddressArrayCount++;
      more = ow.search(ptr);
    }
  }
  owState = OW_CONFIGURED;
}

void owStartConversion()
{
  uint8_t status;

  status = ow.reset();
  if (status == 0) {
    return;
  }

  ow.skip(); // Address all devices
  ow.write(0x44, 1); // Start conversion
  owState = OW_WAIT_FOR_CONVERSION;
}

void owReadTemperature()
{
  uint8_t status;
  uint8_t data[12];
  uint8_t index = owState & 0x7F;
  uint8_t *address = owAddressArray + (index * 8);
  bool crcOk = false;

  status = ow.reset();
  if (status == 0) {
    owState = OW_RESET;
    return;
  }
  status = ow.reset();
  if (status == 0) { return; }
  ow.select(address);
  ow.write(0xBE);         // Read Scratchpad
  for (int i = 0; i < 9; i++) {
    data[i] = ow.read();
  }
  crcOk = data[8] == OneWire::crc8(data, 8);

  if (crcOk && address[0] == 0x28) {
    sentence.clear();
    sentence.print("TMP,"); 
    for (int i = 0; i < 8; i++) {
      ErikLib::HexPrint(sentence, *(address + i));
    }
    sentence.print(",");
    int16_t temp_16 = (int16_t)(data[1]) << 8 | data[0];
    float temp = temp_16 / 16.0f;
    sentence.print(temp, 4);
    Serial.print(sentence);
    setDisplay(temp);
  }
  else {
    Serial.println();
  }
  index++;
  if (index < owAddressArrayCount && index < 0x7f) {
    owState = OW_READ_TEMPERATURE | (index & 0x7f);
  }
  else {
    owState = OW_IDLE;
  }
}

void updateEncoder()
{
  uint8_t MSB = digitalRead(rotA); //MSB = most significant bit
  uint8_t LSB = digitalRead(rotB); //LSB = least significant bit

  uint8_t encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  uint8_t sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
}

void setDisplay(const float value)
{
  if (value > 99.9f || value < -99.9f) {
    return;
  }
  float v = value;
  uint8_t digit;
  uint8_t chars[4] = {0};
  // negative ?
  if (value < 0) {
    chars[0] = 0x2D;
  }
  else {
    chars[0] = 0x10;
  }

  digit = (uint8_t)(v / 10.0f);
  chars[1] = digit;
  v = v - (digit * 10);
  digit = (uint8_t)(v);
  chars[2] = digit;
  v = v - digit;
  digit = (uint8_t)((v * 10.0f) + 0.5f);
  chars[3] = digit;

  Wire.beginTransmission(0x71);
  Wire.write(0x76); // clear
  digit = thermostatMode == TM_LOW ? 0x24 : 0x04;
  Wire.write(0x77); // set dot
  Wire.write(digit); // set dot
  for (uint8_t n = 0; n < 4; n++) {
    Wire.write(chars[n]);
  }
  Wire.endTransmission();
}
