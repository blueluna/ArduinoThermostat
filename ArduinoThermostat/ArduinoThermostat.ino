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
#include <EEPROM.h>

const uint8_t relay1Pin = 7;
const uint8_t relay2Pin = 8;
const uint8_t btnPin = A5;
const uint8_t owPin = 5;
const uint8_t rotA = 10;
const uint8_t rotB = 9;
const uint8_t rotRed = 11;
const uint8_t rotGreen = 13;
const uint8_t rotBtn = 4;

const uint8_t rotGreenLedBrightness = 32;
const uint8_t rotRedLedBrightness = 16;

const uint32_t relaySwitchDelay = 500; // milliseconds
uint32_t relaySwitchTime = 0; // milliseconds
uint8_t relayActive = 0; // the active relay (1 or 2)
uint8_t relayToActivate = 0; // the relay to activate (1 or 2)

OneWire ow(owPin); // The 1-wire bus
uint8_t owAddressArrayCapacity = 0; // capacity for array of 1-wire device addresses
uint8_t *owAddressArray = 0; // array of 1-wire device addresses
uint8_t owAddressArrayCount = 0; // number of devices in array of 1-wire device addresses

uint8_t lastEncoded = 0;
int16_t encoderValue = -10000;
int16_t oldEncoderValue = -10000;
uint32_t encoderUpdates = 0;

enum {
  TM_NORMAL = 0,
  TM_LOW
};
uint8_t thermostatMode = TM_NORMAL;
uint8_t oldThermostatMode = TM_NORMAL;
int16_t thermostatThresholdNormal = 0;
int16_t thermostatThresholdLow = 0;

NMEASentence sentence(128);
NMEAParser parser(128);
Button btnBoard(btnPin, 50);
Button btnRot(rotBtn, 50);

enum {
  OW_RESET = 0,
  OW_CONFIGURED,
  OW_IDLE,
  OW_WAIT_FOR_CONVERSION,
  OW_CONFIGURE_THRESHOLD,
  OW_CHANGE_MODE,
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
  digitalWrite(rotRed, LOW);
  pinMode(rotGreen, OUTPUT); 
  digitalWrite(rotGreen, LOW);

  owLoadConfiguration();

  Wire.begin();
  displayThreshold();

  Serial.begin(115200);

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
  handleRelays(t);
  owHandler(t);
  if (Serial.available()) {
    int16_t result = parser.Parse(Serial);
    if (result == E_SUCCESS) {
      handleSentence();
    }
    else {
      sentence.clear();
      sentence.print("$ERR,");
      sentence.print(result);
      Serial.print(sentence);
    }
  }
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

      sentence.clear();
      sentence.print("RLY,");
      sentence.print(relayActive);
      sentence.comma();
      sentence.print(relayToActivate);
      Serial.print(sentence);

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
  uint8_t state;
  if (encoderValue != oldEncoderValue) {
    if (oldEncoderValue > -10000) {
      owStateTime = millis();
      owState = OW_CONFIGURE_THRESHOLD;
    }
    encoderUpdates++;
  }
  else {
    state = btnRot.Check(t);
    if (state == Button::Falling) {
      owStateTime = millis();
      thermostatMode = (thermostatMode == TM_NORMAL) ? TM_LOW : TM_NORMAL;
      owState = OW_CHANGE_MODE;
    }
  }
  state = owState;
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
    case OW_CONFIGURE_THRESHOLD:
      owConfigureThreshold();
      if (t > (owStateTime + 10000)) {
        owStoreConfiguration();
        owState = OW_IDLE;
      }
      break;
    case OW_CHANGE_MODE:
      owChangeMode();
      if (t > (owStateTime + 10000)) {
        owStoreConfiguration();
        owState = OW_IDLE;
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

  if (encoderValue != oldEncoderValue) {
    oldEncoderValue = encoderValue;
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
    sentence.comma();
    int16_t temp_16 = (int16_t)(data[1]) << 8 | data[0];
    float temp = temp_16 / 16.0f;
    sentence.print(temp, 4);
    Serial.print(sentence);
    displayTemperature(temp, 32);
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

void owConfigureThreshold()
{
  if (encoderValue != oldEncoderValue) {
    if (thermostatMode == TM_LOW) {
      thermostatThresholdLow = encoderValue;
    }
    else {
      thermostatThresholdNormal = encoderValue;
    }
    displayThreshold();
  }
}

void owChangeMode()
{
  if (oldThermostatMode != thermostatMode) {
    displayThreshold();
  }
  oldThermostatMode = thermostatMode;
}

void owLoadConfiguration()
{
  uint8_t dat8_1, dat8_2;
  thermostatMode = EEPROM.read(0x10);
  oldThermostatMode = thermostatMode;
  dat8_1 = EEPROM.read(0x11);
  dat8_2 = EEPROM.read(0x12);
  thermostatThresholdNormal = (int16_t)(dat8_1) << 8 | dat8_2;
  dat8_1 = EEPROM.read(0x13);
  dat8_2 = EEPROM.read(0x14);
  thermostatThresholdLow = (int16_t)(dat8_1) << 8 | dat8_2;

  if (thermostatMode == TM_LOW) {
    encoderValue = thermostatThresholdLow;
  }
  else {
    encoderValue = thermostatThresholdNormal;
  }
}

void owStoreConfiguration()
{
  uint8_t dat8_1, dat8_2;
  int16_t dat16;
  uint8_t changes = 0;
  dat8_1 = EEPROM.read(0x10);
  if (thermostatMode != dat8_1) {
    EEPROM.write(0x10, thermostatMode);
    changes++;
  }
  dat8_1 = EEPROM.read(0x11);
  dat8_2 = EEPROM.read(0x12);
  dat16 = (int16_t)(dat8_1) << 8 | dat8_2;
  if (thermostatThresholdNormal != dat16) {
    EEPROM.write(0x11, (uint8_t)(thermostatThresholdNormal >> 8));
    EEPROM.write(0x12, (uint8_t)(thermostatThresholdNormal));
    changes++;
  }
  dat8_1 = EEPROM.read(0x13);
  dat8_2 = EEPROM.read(0x14);
  dat16 = (int16_t)(dat8_1) << 8 | dat8_2;
  if (thermostatThresholdLow != dat16) {
    EEPROM.write(0x13, (uint8_t)(thermostatThresholdLow >> 8));
    EEPROM.write(0x14, (uint8_t)(thermostatThresholdLow));
    changes++;
  }
  displayClear();

  if (changes > 0) {
    sendCFGSentence();
  }
}

void updateEncoder()
{
  uint8_t MSB = digitalRead(rotA); // MSB = most significant bit
  uint8_t LSB = digitalRead(rotB); // LSB = least significant bit

  uint8_t encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
  uint8_t sum  = (lastEncoded << 2) | encoded; // adding it to the previous encoded value

  if (thermostatMode == TM_LOW) {
    encoderValue = thermostatThresholdLow;
  }
  else {
    encoderValue = thermostatThresholdNormal;
  }

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  if (encoderValue > 999) {
    encoderValue = 999;
  }
  else if (encoderValue < -999) {
    encoderValue = -999;
  }

  lastEncoded = encoded; // store this value for next time
}

void displayTemperature(const float value, const uint8_t brightness)
{
  displayFloat(value, brightness);
  digitalWrite(rotRed, LOW);
  digitalWrite(rotGreen, LOW);
}

void displayFloat(const float value, const uint8_t brightness)
{
  if (value > 99.9f || value < -99.9f) {
    return;
  }
  float v = value;
  uint8_t digit;
  uint8_t chars[4] = {0};
  // negative ?
  if (v < 0) {
    chars[0] = 0x2D;
    v = -v;
  }
  else {
    chars[0] = 0x10;
  }

  digit = (uint8_t)(v / 10.0f);
  if (digit == 0) {
    chars[1] = chars[0];
    chars[0] = 0x10;
  }
  else {
    chars[1] = digit;
  }
  v = v - (digit * 10);
  digit = (uint8_t)(v);
  chars[2] = digit;
  v = v - digit;
  digit = (uint8_t)((v * 10.0f) + 0.5f);
  chars[3] = digit;

  Wire.beginTransmission(0x71);
  Wire.write(0x76); // clear
  Wire.write(0x7A);  // brightness
  Wire.write(brightness);
  digit = thermostatMode == TM_LOW ? 0x24 : 0x04;
  Wire.write(0x77); // set dot
  Wire.write(digit); // set dot
  for (uint8_t n = 0; n < 4; n++) {
    Wire.write(chars[n]);
  }
  Wire.endTransmission();
}

void displayThreshold()
{
  int16_t threshold = 0;
  float temp = 0.0f;
  if (thermostatMode == TM_LOW) {
    threshold = thermostatThresholdLow;
    analogWrite(rotGreen, rotGreenLedBrightness);
    digitalWrite(rotRed, LOW);
  }
  else {
    threshold = thermostatThresholdNormal;
    analogWrite(rotRed, rotRedLedBrightness);
    digitalWrite(rotGreen, LOW);
  }
  temp = threshold / 10.0f;
  displayFloat(temp, 192);
}

void displayClear()
{
  Wire.beginTransmission(0x71);
  Wire.write(0x76); // clear
  Wire.endTransmission();
  digitalWrite(rotRed, LOW);
  digitalWrite(rotGreen, LOW);
}

void handleSentence()
{
  uint8_t tokens = parser.Tokens();
  uint8_t n = 0;
  int16_t len = 0;
  if (tokens == 0) {
    return;
  }
  const char* token = reinterpret_cast<const char*>(parser.Token(n, len));
  if (len != 3) {
    return;
  }
  if (strncmp(token, "CFG", 3) == 0 && tokens == 4) {
    if (tokens == 4) {
      handleCFGSentence(n+1);
    }
    else {
      sendCFGSentence();
    }
  }
  /*
  for (uint8_t n = 0; n < parser.Tokens(); n++) {
    int16_t len = 0;
    const uint8_t* token = parser.Token(n, len);
  }
  sentence.clear();
  sentence.write(parser.GetSentence().Peek(), parser.GetSentence().Used());
  Serial.print(sentence);
  */
}

void handleCFGSentence(const uint8_t n)
{
  int16_t len = 0;
  int16_t mode, normalThreshold, lowThreshold;
  const char* token;
  token = reinterpret_cast<const char*>(parser.Token(n, len));
  mode = strtoul(token, 0, 10);
  token = reinterpret_cast<const char*>(parser.Token(n+1, len));
  normalThreshold = strtol(token, 0, 10);
  token = reinterpret_cast<const char*>(parser.Token(n+2, len));
  lowThreshold = strtol(token, 0, 10);

  sentence.clear();
  sentence.print("CFG,");
  sentence.print(mode);
  sentence.print(',');
  sentence.print(normalThreshold);
  sentence.print(',');
  sentence.print(lowThreshold);
  Serial.print(sentence);
}

void sendCFGSentence()
{
  sentence.clear();
  sentence.print("CFG,"); 
  sentence.print(thermostatMode);
  sentence.comma();
  sentence.print(thermostatThresholdNormal);
  sentence.comma();
  sentence.print(thermostatThresholdLow);
  Serial.print(sentence);
}