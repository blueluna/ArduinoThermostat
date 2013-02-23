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

const uint32_t relaySwitchDelay = 900000; // 15 minutes in milliseconds
uint32_t relaySwitchTime = 0; // milliseconds
uint8_t relayState = 0; // the relay state
int16_t temperatureAccumulator = 0;
uint8_t readoutCount = 0;
int16_t temperatureAverage = 0;

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
  TM_LOW,
  TM_END
};
uint8_t thermostatMode = TM_NORMAL;
uint8_t oldThermostatMode = TM_NORMAL;
int16_t thermostatThresholdNormal = 0;
int16_t thermostatThresholdLow = 0;
int8_t hysteresisUpper = 5;
int8_t hysteresisLower = -5;
uint8_t masterSensor[8] = {0};
int8_t masterFound = 0;

NMEASentence sentence(128);
NMEAParser parser(128);
Button btnBoard(btnPin, 50);
Button btnRot(rotBtn, 50);

float temperature = -100.0;
float temperatureOld = -100.0;

enum {
  OW_RESET = 0,
  OW_CONFIGURED,
  OW_IDLE,
  OW_WAIT_FOR_CONVERSION,
  OW_READ_TEMPERATURE = 0x80
};

uint8_t owState = OW_RESET; // current state of the 1-wire state machine
uint32_t owStateTime = 0; // time for the last state change

enum {
  DS_IDLE = 0,
  DS_CONFIGURE_THRESHOLD,
  DS_CHANGE_MODE
};

uint8_t dsState = OW_RESET; // current state of the 1-wire state machine
uint32_t dsStateTime = 0; // time for the last state change

void setup()
{
  uint32_t t = millis();

  pinMode(relay1Pin, OUTPUT); 
  digitalWrite(relay1Pin, LOW);
  pinMode(relay2Pin, OUTPUT); 
  digitalWrite(relay2Pin, LOW);
  relaySwitchTime = t;
  relayState = 0;

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

  loadConfiguration();

  Wire.begin();
  displayThreshold();

  Serial1.begin(115200);

  sendCFGSentence();

  owState = OW_RESET;
  owStateTime = t;
  dsState = DS_CONFIGURE_THRESHOLD;
  dsStateTime = t;
  owHandler(t);
  dsHandler(t);

  PCintPort::attachInterrupt(rotA, &updateEncoder, CHANGE);
  PCintPort::attachInterrupt(rotB, &updateEncoder, CHANGE);
}

void loop()
{
  uint8_t state;
  uint32_t t = millis();
  handleRelays(t);
  owHandler(t);
  dsHandler(t);
  if (Serial1.available()) {
    int16_t result = parser.Parse(Serial1);
    if (result == E_SUCCESS) {
      handleSentence();
    }
    else {
      sendERRSentence(result);
    }
  }
}

void handleRelays(const uint32_t t)
{
  if (t >= (relaySwitchTime + relaySwitchDelay)) {
    relaySwitchTime = t;
    int16_t threshold;
    switch (thermostatMode) {
      case TM_LOW:
        threshold = thermostatThresholdLow;
        break;
      default:
        threshold = thermostatThresholdNormal;
        break;
    }
    if (relayState == 1) {
      if (temperatureAverage > (threshold + hysteresisUpper)) {
        relayState = 0;
        digitalWrite(relay1Pin, LOW);
      }
    }
    else {
      if (temperatureAverage < (threshold + hysteresisLower)) {
        relayState = 1;
        digitalWrite(relay1Pin, HIGH);
      }
    }
    sendCTLSentence();
  }
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
        owStateTime = t;
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
    masterFound = 0;
    uint8_t *ptr = owAddressArray;
    more = ow.search(ptr);
    for (uint8_t n = 0; n < count && more == 1; n++) {
      sendSCNSentence(ptr);
      if (memcmp(ptr, masterSensor, 8) == 0) {
        sendMTRSentence(ptr);
        masterFound = 1;
      }
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

void calulateAverage(int16_t temp)
{
  temperatureAccumulator += (int16_t)(temp / 1.6f);
  readoutCount++;
  if (readoutCount == 32) {
    temperatureAverage = (temperatureAccumulator / 32);
    temperatureAccumulator = 0;
    readoutCount = 0;
    sendAVGSentence();
  }
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
    int16_t temp_16 = (int16_t)(data[1]) << 8 | data[0];
    float temp = temp_16 / 16.0f;
    sendTMPSentence(address, temp);
    if (masterFound == 1) {
      if (memcmp(address, masterSensor, 8) == 0) {
        calulateAverage(temp_16);
        temperature = temp;
      }
    }
    else {
      if (index == 0) {
        calulateAverage(temp_16);
        temperature = temp;
      }
    }
  }
  index++;
  if (index < owAddressArrayCount && index < 0x7f) {
    owState = OW_READ_TEMPERATURE | (index & 0x7f);
  }
  else {
    owState = OW_IDLE;
  }
}

void dsHandler(const uint32_t t)
{
  uint8_t state;
  if (encoderValue != oldEncoderValue) {
    if (oldEncoderValue > -10000) {
      dsStateTime = t;
      dsState = DS_CONFIGURE_THRESHOLD;
    }
    encoderUpdates++;
  }
  else {
    state = btnRot.Check(t);
    if (state == Button::Falling) {
      dsStateTime = t;
      thermostatMode = (thermostatMode == TM_NORMAL) ? TM_LOW : TM_NORMAL;
      dsState = DS_CHANGE_MODE;
    }
  }

  switch (dsState) {
    case DS_IDLE:
      if (temperatureOld != temperature) {
        displayTemperature(temperature, 32);
      }
      temperatureOld = temperature;
      break;
    case DS_CONFIGURE_THRESHOLD:
      configureThreshold();
      if (t > (dsStateTime + 10000)) {
        storeConfiguration();
        dsState = DS_IDLE;
      }
      break;
    case DS_CHANGE_MODE:
      changeMode();
      if (t > (dsStateTime + 10000)) {
        storeConfiguration();
        dsState = DS_IDLE;
      }
      break;
  }

  if (encoderValue != oldEncoderValue) {
    oldEncoderValue = encoderValue;
  }
}

void configureThreshold()
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

void changeMode()
{
  if (oldThermostatMode != thermostatMode) {
    displayThreshold();
  }
  oldThermostatMode = thermostatMode;
}

void loadConfiguration()
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
  
  dat8_1 = EEPROM.read(0x15);
  hysteresisUpper = (int8_t)dat8_1;
  dat8_1 = EEPROM.read(0x16);
  hysteresisLower = (int8_t)dat8_1;
  masterSensor[0] = EEPROM.read(0x18);
  masterSensor[1] = EEPROM.read(0x19);
  masterSensor[2] = EEPROM.read(0x1a);
  masterSensor[3] = EEPROM.read(0x1b);
  masterSensor[4] = EEPROM.read(0x1c);
  masterSensor[5] = EEPROM.read(0x1d);
  masterSensor[6] = EEPROM.read(0x1e);
  masterSensor[7] = EEPROM.read(0x1f);
}

void storeConfiguration()
{
  uint8_t dat8_1, dat8_2;
  int8_t di8;
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
  di8 = (int8_t)(EEPROM.read(0x15));
  if (di8 != hysteresisUpper) {
    EEPROM.write(0x15, (uint8_t)(hysteresisUpper));
  }
  di8 = (int8_t)(EEPROM.read(0x16));
  if (di8 != hysteresisLower) {
    EEPROM.write(0x16, (uint8_t)(hysteresisLower));
  }
  for (int8_t n = 0; n < 8; n++) {
    dat8_1 = EEPROM.read(0x18 + n);
    if (masterSensor[n] != dat8_1) {
      EEPROM.write(0x18 + n, masterSensor[n]);
    }
  }
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
  n++;
  if (strncmp(token, "CFG", 3) == 0) {
    if (tokens == 7) {
      handleCFGSentence(n);
    }
    else {
      sendCFGSentence();
    }
  }
  else if (strncmp(token, "SCN", 3) == 0) {
    owScanBus();
  }
  else if (strncmp(token, "CTL", 3) == 0) {
    sendCTLSentence();
  }
}

void handleCFGSentence(const uint8_t n)
{
  int16_t len = 0;
  int16_t mode, normalThreshold, lowThreshold;
  int8_t hystUpper, hystLower;
  uint32_t u32;
  uint8_t address[8] = {0};
  char part[3] = {0};
  const char* token;
  token = reinterpret_cast<const char*>(parser.Token(n, len));
  mode = strtoul(token, 0, 10);
  token = reinterpret_cast<const char*>(parser.Token(n+1, len));
  normalThreshold = strtol(token, 0, 10);
  token = reinterpret_cast<const char*>(parser.Token(n+2, len));
  lowThreshold = strtol(token, 0, 10);
  token = reinterpret_cast<const char*>(parser.Token(n+3, len));
  hystUpper = strtol(token, 0, 10);
  token = reinterpret_cast<const char*>(parser.Token(n+4, len));
  hystLower = strtol(token, 0, 10);
  token = reinterpret_cast<const char*>(parser.Token(n+5, len));
  for (uint8_t n = 0; n < 8; n++) {
    memcpy(part, token, 2);
    part[2] = 0;
    u32 = strtoul(part, 0, 16);
    address[n] = u32 & 0xff;
    token += 2; 
  }

  uint8_t cfgOk = 0;
  if (mode >= 0 && mode < TM_END) { cfgOk++; }
  if (normalThreshold >= -999 && normalThreshold <= 999) { cfgOk++; }
  if (lowThreshold >= -999 && lowThreshold <= 999) { cfgOk++; }
  if (hystUpper >= -125 && hystUpper <= 125) { cfgOk++; }
  if (hystLower >= -125 && hystLower <= 125) { cfgOk++; }
  if (address[0] == 0x28) { cfgOk++; }

  if (cfgOk == 6) {
    thermostatMode = mode;
    thermostatThresholdNormal = normalThreshold;
    thermostatThresholdLow = lowThreshold;
    hysteresisUpper = hystUpper;
    hysteresisLower = hystLower;
    memcpy(masterSensor, address, 8);
    masterFound = 0;
    storeConfiguration();
    owScanBus();
  }
  sendCFGSentence();
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
  sentence.comma();
  sentence.print(hysteresisUpper);
  sentence.comma();
  sentence.print(hysteresisLower);
  sentence.comma();
  printAddress(sentence, masterSensor);
  Serial1.print(sentence);
}

void sendSCNSentence(const uint8_t address[8])
{
  sentence.clear();
  sentence.print("SCN,");
  printAddress(sentence, address);
  Serial1.print(sentence);
}

void sendMTRSentence(const uint8_t address[8])
{
  sentence.clear();
  sentence.print("MTR,");
  printAddress(sentence, address);
  Serial1.print(sentence);
}

void sendTMPSentence(const uint8_t address[8], const float temp)
{
  sentence.clear();
  sentence.print("TMP,"); 
  printAddress(sentence, address);
  sentence.comma();
  sentence.print(temp, 4);
  Serial1.print(sentence);
}

void sendCTLSentence()
{
  sentence.clear();
  sentence.print("CTL,");
  sentence.print(relayState);
  Serial1.print(sentence);
}

void sendERRSentence(const int32_t code)
{
  sentence.clear();
  sentence.print("ERR,");
  sentence.print(code);
  Serial1.print(sentence);
}

void sendAVGSentence()
{
  sentence.clear();
  sentence.print("AVG,");
  sentence.print(temperatureAverage);
  Serial1.print(sentence);
}

void printAddress(Print& printer, const uint8_t address[8])
{
  for (int i = 0; i < 8; i++) {
    ErikLib::HexPrint(printer, *(address + i));
  }
}