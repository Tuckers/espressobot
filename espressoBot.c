#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_IS31FL3731.h>
#include <EEPROM.h>

#define BRIGHTNESS 64
#define PUMP_OUTPUT_PIN 47
#define INLET_OUTPUT_PIN 49

#define LEFT_OUTPUT_MODE LOW
#define LEFT_OUTPUT_PIN 51
#define LEFT_MODE_PIN 22
#define LEFT_SELECT_PIN 24
#define LEFT_PLUS_PIN 26
#define LEFT_MINUS_PIN 28

#define RIGHT_OUTPUT_MODE HIGH
#define RIGHT_OUTPUT_PIN 53
#define RIGHT_MODE_PIN 31
#define RIGHT_SELECT_PIN 33
#define RIGHT_PLUS_PIN 35
#define RIGHT_MINUS_PIN 37

#define DATA_SET_BOOL_ADDRESS 0
#define DATA_ADDRESS 1




int pumpState = HIGH;
int mode = 0;
unsigned long debounceTime = 20;
unsigned long longPress = 1000;
int modes = 5;

static const uint8_t PROGMEM
zero_bmp[] =
{ B11110000,
  B10010000,
  B10010000,
  B10010000,
  B10010000,
  B10010000,
  B10010000,
  B10010000,
  B11110000
},
one_bmp[] =
{ B00100000,
  B01100000,
  B00100000,
  B00100000,
  B00100000,
  B00100000,
  B00100000,
  B00100000,
  B00100000
},
two_bmp[] =
{ B11110000,
  B00010000,
  B00010000,
  B01110000,
  B10000000,
  B10000000,
  B10000000,
  B10000000,
  B11110000
},
three_bmp[] =
{ B11110000,
  B00010000,
  B00010000,
  B11100000,
  B00010000,
  B00010000,
  B00010000,
  B00010000,
  B11110000
},
four_bmp[] =
{ B00010000,
  B00110000,
  B01010000,
  B11110000,
  B00010000,
  B00010000,
  B00010000,
  B00010000,
  B00010000
},
five_bmp[] =
{ B11110000,
  B10000000,
  B10000000,
  B11110000,
  B00010000,
  B00010000,
  B00010000,
  B00010000,
  B11100000
},
six_bmp[] =
{ B01110000,
  B10000000,
  B10000000,
  B11110000,
  B10010000,
  B10010000,
  B10010000,
  B10010000,
  B11110000
},
seven_bmp[] =
{ B11110000,
  B00010000,
  B00100000,
  B01000000,
  B1000000,
  B10000000,
  B10000000,
  B10000000,
  B10000000
},
eight_bmp[] =
{ B11110000,
  B10010000,
  B10010000,
  B01100000,
  B10010000,
  B10010000,
  B10010000,
  B10010000,
  B11110000
},
nine_bmp[] =
{ B11110000,
  B10010000,
  B10010000,
  B01110000,
  B00010000,
  B00010000,
  B00010000,
  B00010000,
  B00010000
},
cup_bmp[] =
{ B11111000,
  B10001000,
  B10011000,
  B11110000
},
rain1_bmp[] =
{ B10000000,
  B00010000,
  B01000000,
  B00000000,
  B00100000
},
rain2_bmp[] =
{ B00000000,
  B10000000,
  B00010000,
  B01000000,
  B00000000
},
rain3_bmp[] =
{ B00100000,
  B00000000,
  B10000000,
  B00010000,
  B01000000
},
rain4_bmp[] =
{ B00000000,
  B00100000,
  B00000000,
  B10000000,
  B00010000
},
rain5_bmp[] =
{ B01000000,
  B00000000,
  B00100000,
  B00000000,
  B10000000
},
rain6_bmp[] =
{ B00010000,
  B01000000,
  B00000000,
  B00100000,
  B00000000
},
modeM_bmp[] =
{ B11011000,
  B10101000,
  B10101000,
  B10001000,
  B10001000
},
modeT_bmp[] =
{ B11111000,
  B00100000,
  B00100000,
  B00100000,
  B00100000
},
modeC_bmp[] =
{ B01111000,
  B10000000,
  B10000000,
  B10000000,
  B01111000
},
modeV_bmp[] =
{ B10001000,
  B10001000,
  B10001000,
  B01010000,
  B00100000
},
modeF_bmp[] =
{ B11111000,
  B10000000,
  B11100000,
  B10000000,
  B10000000
},
modeS_bmp[] =
{ B01111000,
  B10000000,
  B01110000,
  B00001000,
  B11110000
},
modeTemp_bmp[] =
{ B11100000,
  B10100000,
  B11100000,
  B00000000,
  B00000000,
  B11100000,
  B10000000,
  B10000000,
  B11100000
};

const uint8_t *const numbers[] PROGMEM = {zero_bmp, one_bmp, two_bmp, three_bmp, four_bmp, five_bmp, six_bmp, seven_bmp, eight_bmp, nine_bmp};

Adafruit_IS31FL3731 leftMatrix = Adafruit_IS31FL3731();
Adafruit_IS31FL3731 rightMatrix = Adafruit_IS31FL3731();

///// STUCT DEFINTIONS /////

typedef struct GroupStore_t {
  int timeRecord;
  int volumeRecord;
} GroupStore_t;

typedef struct StoredData_t {
  GroupStore_t leftGroup;
  GroupStore_t rightGroup;
} StoredData_t;

typedef struct Input_t {
  const int pin;
  bool isActive;
  bool isActiveLong;
  int prevReading;
  unsigned long timer;
} InputPush_t;

typedef struct Flowmeter_t {
  const int pin;
  volatile uint16_t pulses;
  volatile uint32_t lastflowratetimer;
  volatile uint8_t lastflowpinstate;
  volatile float flowrate;
} Flowmeter_t;

typedef struct Group_t {
  String name;
  int mode;
  Input_t modeButton;
  Input_t selectButton;
  Input_t plusButton;
  Input_t minusButton;
  bool isOutputting;
  const int outputPin;
  const int outputMode;
  unsigned long startTime;
  int totalElapsed;
  Flowmeter_t flowmeter;
  Adafruit_IS31FL3731 matrix;
  const int matrixAddress;
  int currentFrame;
  long timeRecord;
  float volumeRecord;
  int cleaningCycle;
  bool isCleaning;
  bool isPaused;
} Group_t;

///// GROUP SETUP /////

Group_t rightGroup = {
  "Right",
  3, // CURRENT MODE
  Input_t {RIGHT_MODE_PIN, false, false, true}, // MODE BUTTON
  Input_t {RIGHT_SELECT_PIN, false, false, true}, // SELECT BUTTON
  Input_t {RIGHT_PLUS_PIN, false, false, true}, // PLUS BUTTON
  Input_t {RIGHT_MINUS_PIN, false, false, true},  // MINUS BUTTON
  false, // is outputting
  RIGHT_OUTPUT_PIN, // Output Pin
  RIGHT_OUTPUT_MODE, // Output Mode
  0, // Start time
  0, // Total Elapsed
  Flowmeter_t {3, 0, 0}, // Flow Meter Input
  rightMatrix, // Matrix
  0x75, // Matrix Address
  0, // Current Frame
  28000, // Time Record
  0, // Volume Record
  0, // Cleaning Cycle
  false, // isCleaning
  false // isPaused
};

Group_t leftGroup = {
  "Left",
  3, // CURRENT MODE
  Input_t {LEFT_MODE_PIN, false, false, true}, // MODE BUTTON
  Input_t {LEFT_SELECT_PIN, false, false, true}, // SELECT BUTTON
  Input_t {LEFT_PLUS_PIN, false, false, true}, // PLUS BUTTON
  Input_t {LEFT_MINUS_PIN, false, false, true},  // MINUS BUTTON
  false, // is outputting
  LEFT_OUTPUT_PIN, // OUTPUT
  RIGHT_OUTPUT_MODE, // Output Mode
  0, // Start time
  0, // Total Elapsed
  Flowmeter_t {2, 0, 0}, // Flow Meter Input
  leftMatrix, // Matrix
  0x74, // Matrix Address
  0, // Current Frame
  28000, // Time Record
  0, // Volume Record
  0,
  false,
  false
};

///// FLOW METER /////

// Looks for any pulses from the flowmeter sensor
void readFlowmeter(Flowmeter_t *flowmeter) {
  uint8_t x = digitalRead(flowmeter->pin);

  if (x == flowmeter->lastflowpinstate) {
    flowmeter->lastflowratetimer++;
    return; // nothing changed!
  }

  if (x == HIGH) {
    //low to high transition!
    flowmeter->pulses++;
  }
  flowmeter->lastflowpinstate = x;
  flowmeter->flowrate = 1000.0;
  flowmeter->flowrate /= flowmeter->lastflowratetimer;  // in hertz
  flowmeter->lastflowratetimer = 0;
}

// Interrupt is called once a millisecond
SIGNAL(TIMER0_COMPA_vect) {
  readFlowmeter(&rightGroup.flowmeter);
  readFlowmeter(&leftGroup.flowmeter);
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
  }
}

void getFlowPinState(uint8_t *flowpinstate, Group_t group) {
  *flowpinstate = digitalRead(group.flowmeter.pin);
}

//// PIN SETUP /////

void setupGroup(Group_t group) {
  pinMode(group.modeButton.pin, INPUT_PULLUP);
  pinMode(group.selectButton.pin, INPUT_PULLUP);
  pinMode(group.plusButton.pin, INPUT_PULLUP);
  pinMode(group.minusButton.pin, INPUT_PULLUP);
  pinMode(group.outputPin, OUTPUT);
  digitalWrite(group.outputPin, HIGH);
  pinMode(group.flowmeter.pin, INPUT);
  digitalWrite(group.flowmeter.pin, HIGH);
  getFlowPinState(&group.flowmeter.lastflowpinstate, group);
}

///// SETUP /////

void setup() {
  Serial.begin(9600);
  while (!Serial) {
  }
  setupGroup(leftGroup);
  setupGroup(rightGroup);
  pinMode(PUMP_OUTPUT_PIN, OUTPUT);
  pinMode(INLET_OUTPUT_PIN, OUTPUT);
  useInterrupt(true);
  leftMatrix.begin();
  leftGroup.matrix = leftMatrix;
  rightMatrix.begin(0x75);
  rightGroup.matrix = rightMatrix;
  bool firstTime;
  EEPROM.get(DATA_SET_BOOL_ADDRESS, firstTime);
  if (firstTime) {
    firstTime = false;
    Serial.println("It's the first time, setting initial values to EEPROM!");
    EEPROM.put(DATA_SET_BOOL_ADDRESS, firstTime);
    //EEPROM.put(DATA_ADDRESS, savedData);
  } else {
    Serial.println("Not my first Rodeo");
  }
}


///// LOOP /////
void loop() {
  flipFrames(&leftGroup);
  flipFrames(&rightGroup);

  //CHECK INPUTS//
  getGroupState(&leftGroup);
  getGroupState(&rightGroup);

  setGroupMode(&leftGroup);
  setGroupMode(&rightGroup);

  handleInput(&leftGroup);
  handleInput(&rightGroup);

  displayMode(&leftGroup);
  displayMode(&rightGroup);

  //printGroupState(&leftGroup);
  printGroupState(&rightGroup);

  if (leftGroup.isOutputting || rightGroup.isOutputting) {
    pumpState = LOW;
  } else {
    pumpState = HIGH;
  }

  digitalWrite(PUMP_OUTPUT_PIN, pumpState);
  digitalWrite(INLET_OUTPUT_PIN, pumpState);

}

void getPushState(Input_t *input) {
  bool reading = digitalRead(input->pin);
  long duration = millis() - input->timer;
  if (reading == false && input->prevReading == true) {
    input->timer = millis();
  } else if (reading == false && input->prevReading == false) {
    if (duration > longPress) {
      input->isActiveLong = true;
    }
  } else if (reading == true && input->prevReading == false) {
    if (duration > debounceTime && duration < longPress) {
      input->isActive = true;
    }
  } else if (reading == true && input->prevReading == true) {
    input->isActive = false;
    input->isActiveLong = false;
  }
  input->prevReading = reading;
}

void getGroupState(Group_t *group) {
  getPushState(&group->modeButton);
  getPushState(&group->selectButton);
  getPushState(&group->plusButton);
  getPushState(&group->minusButton);
}

void printGroupState(Group_t *group) {
  Serial.println("------------");
  Serial.println(group->name);
  Serial.println("------------");
  Serial.print("Current mode: ");
  Serial.println(group->mode);
  Serial.print("Outputting: ");
  Serial.println(group->isOutputting ? "HIGH" : "LOW");
  Serial.println("BUTTONS");
  Serial.print("Mode: ");
  printButtonState(&group->modeButton);
  Serial.print("Select: ");
  printButtonState(&group->selectButton);
  Serial.print("Plus: ");
  printButtonState(&group->plusButton);
  Serial.print("Minus: ");
  printButtonState(&group->minusButton);
  Serial.println("");
}

void setGroupMode(Group_t *group) {
  if (group->modeButton.isActive == true) {
    group->mode++;
    if (group->mode > modes) {
      group->mode = 0;
    }
    if (group->isOutputting) {
      turnOffGroup(group);
    }
  }
}

void printButtonState(Input_t *input) {
  if (input->isActive == true) {
    Serial.print("Active ");
  }
  if (input->isActiveLong == true) {
    Serial.print("Long ");
  }
  Serial.println(input->prevReading ? "" : "Pressed");
}

void displayMode(Group_t *group) {
  //group->matrix.clear();
  long elapsedTime;
  switch (group->mode) {
    case 0:
      // Temperature
      drawNumber(-1, 0, 1, BRIGHTNESS, group->matrix);
      drawNumber(3, 0, 0, BRIGHTNESS, group->matrix);
      drawNumber(8, 0, 2, BRIGHTNESS, group->matrix);
      group->matrix.drawBitmap(13, 0, modeTemp_bmp, 3, 9, BRIGHTNESS);
      break;
    case 1:
      // Manual
      if (group->isOutputting) {
        long elapsedTime = (millis() - group->startTime) / 1000;
        printTime(elapsedTime, group);
      } else {
        drawNumber(0, 0, 0, BRIGHTNESS, group->matrix);
      }
      group->matrix.drawBitmap(11, 0, modeM_bmp, 5, 5, BRIGHTNESS);
      break;
    case 2:
      // Volume
      group->matrix.drawBitmap(11, 0, modeV_bmp, 5, 5, BRIGHTNESS);
      break;
    case 3:
      // Time
      if(group->isOutputting){
        elapsedTime = (millis() - group->startTime) / 100;
        printTimeTenths(elapsedTime, group, true);
      } else {
        if (group->isPaused) {
          printTimeTenths(group->totalElapsed / 100, group, true);
        } else {
        printTimeTenths(group->timeRecord / 100, group, true);
        }
      }
      break;
    case 4:
      // Clean
      group->matrix.setTextSize(1);
      group->matrix.setTextColor(0);
      group->matrix.setCursor(10,1);
      if (group->isCleaning) {
        elapsedTime = (millis() - group->startTime) / 1000;
        if(group->isPaused){
          group->matrix.fillRect(9, 0, 7, 11, BRIGHTNESS);
          group->matrix.print("P");
        } else {
          printTime(elapsedTime, group);
          group->matrix.fillRect(9, 0, 7, 11, BRIGHTNESS);
          if(group->cleaningCycle < 5){
            group->matrix.print(group->cleaningCycle + 1);
          } else {
            group->matrix.print(group->cleaningCycle - 5);
          }
        }
      } else {
        group->matrix.drawBitmap(11, 0, modeC_bmp, 5, 5, BRIGHTNESS);
      }
      break;
    case 5:
      // Sync
      group->matrix.drawBitmap(11, 0, modeS_bmp, 5, 5, BRIGHTNESS);
      break;
  }
  //drawNumber(0, 0, group->mode, 64, group->matrix);
}

void handleInput(Group_t *group) {
  long elapsedTime;
  //group->matrix.clear();
  switch (group->mode) {
    case 0:
      // Temperature
      break;
    case 1:
      // Manual
      if (group->selectButton.isActive) {
        if (group->isOutputting) {
          turnOffGroup(group);
        } else {
          turnOnGroup(group);
        }
      }
      break;
    case 2:
      // Volume
      break;
    case 3:
      // Time
      if(group->isOutputting == false){
        if (group->plusButton.isActive) { //PLUS BUTTON INCREMENT
          group->timeRecord += 100;
        }
        if (group->minusButton.isActive) { //MINUS BUTTON DECREMENT
          group->timeRecord -= 100;
        }
        if (group->selectButton.isActiveLong && group->isPaused){ //LONG PRESS CLEARS
            turnOffGroup(group);
        }
        if (group->selectButton.isActive) {
          if (group->isPaused){ //IF PAUSED RESUME
            resumeGroup(group);
          } else {
            turnOnGroup(group); //ELSE START
          }
        }
      } else {
        if (group->selectButton.isActive) {
          pauseGroup(group); //PAUSE
        }
        if (group->selectButton.isActiveLong){
          turnOffGroup(group);
        }
      }
      
      elapsedTime = (millis() - group->startTime);
      if (elapsedTime >= group->timeRecord){
        turnOffGroup(group);
      }
      break;
    case 4:
      // Clean
      elapsedTime = (millis() - group->startTime) / 1000;

      if (group->isCleaning == false){ // IF NOT CLEANING
        if (group->selectButton.isActive) { // TURN ON
            group->cleaningCycle = 0;
            group->isCleaning = true;
            turnOnGroup(group);
        }
      } else {
          if (group->selectButton.isActive){
              if (group->isPaused){
                  resumeGroup(group);
                  group->cleaningCycle++;
              } else {
                  group->isCleaning = false;
                  turnOffGroup(group);
              }
          }
          if (group->cleaningCycle == 5){
              pauseGroup(group);
          } else {
            if (elapsedTime > 9) {
              if (group->isOutputting) {
                  turnOffGroup(group);
                  group->startTime = millis();
                  if (group->cleaningCycle == 10){
                      group->isCleaning = false;
                }
              } else {
                  group->cleaningCycle++;
                  turnOnGroup(group);
              }
            }
          }
      }
      break;
    case 5:
      // Sync
      break;
  }
  //drawNumber(0, 0, group->mode, 64, group->matrix);
}

void turnOffGroup(Group_t *group) {
  digitalWrite(group->outputPin, HIGH);
  group->isOutputting = false;
  group->isPaused = false;
}

void turnOnGroup(Group_t *group) {
  digitalWrite(group->outputPin, LOW);
  group->isOutputting = true;
  group->startTime = millis();
}

void pauseGroup(Group_t *group) {
  digitalWrite(group->outputPin, HIGH);
  group->isOutputting = false;
  group->isPaused = true;
  group->totalElapsed = millis() - group->startTime;
}

void resumeGroup(Group_t *group) {
  digitalWrite(group->outputPin, LOW);
  group->isOutputting = true;
  group->isPaused = false;
  group->startTime = millis() - group->totalElapsed;
}


///// DISPLAY FUNCTIONS /////
void flipFrames(Group_t *group) {
  group->matrix.displayFrame(group->currentFrame);
  group->currentFrame++;
  if (group->currentFrame > 7) {
    group->currentFrame = 0;
  }
  group->matrix.setFrame(group->currentFrame);
  group->matrix.fillRect(0, 0, 16, 9, 0);
}

void drawNumber(int x, int y, int n, int brightnessVal, Adafruit_IS31FL3731 matrix) {
  switch (n) {
    case 0:
      matrix.drawBitmap(x, y, numbers[0], 5, 9, brightnessVal);
      break;
    case 1:
      matrix.drawBitmap(x, y, numbers[1], 5, 9, brightnessVal);
      break;
    case 2:
      matrix.drawBitmap(x, y, numbers[2], 5, 9, brightnessVal);
      break;
    case 3:
      matrix.drawBitmap(x, y, numbers[3], 5, 9, brightnessVal);
      break;
    case 4:
      matrix.drawBitmap(x, y, numbers[4], 5, 9, brightnessVal);
      break;
    case 5:
      matrix.drawBitmap(x, y, numbers[5], 5, 9, brightnessVal);
      break;
    case 6:
      matrix.drawBitmap(x, y, numbers[6], 5, 9, brightnessVal);
      break;
    case 7:
      matrix.drawBitmap(x, y, numbers[7], 5, 9, brightnessVal);
      break;
    case 8:
      matrix.fillRect(x, y, 5, 9, 0);
      matrix.drawBitmap(x, y, numbers[8], 5, 9, brightnessVal);
      break;
    case 9:
      matrix.drawBitmap(x, y, numbers[9], 5, 9, brightnessVal);
      break;
    default:
      matrix.drawBitmap(x, y, numbers[n], 5, 9, brightnessVal);
      break;
  }
}

void printTime(int elapsed, Group_t *group) {
  int firstDigit;
  int secondDigit;
  if (elapsed < 10) {
    firstDigit = elapsed;
  } else if (elapsed >= 10 && elapsed < 100) {
    secondDigit = elapsed % 10;
    firstDigit = (elapsed - secondDigit) / 10;
    if (elapsed < 20) {
      drawNumber(5, 0, secondDigit, BRIGHTNESS, group->matrix);
    } else {
      drawNumber(5, 0, secondDigit, BRIGHTNESS, group->matrix);
    }
  } else {
    drawNumber(5, 0, 0, BRIGHTNESS, group->matrix);
  }
  drawNumber(0, 0, firstDigit, BRIGHTNESS, group->matrix);
}

void printTimeTenths(int elapsed, Group_t *group, bool showDecimal) {
  int tens;
  int seconds;
  int tenths;
  tenths = elapsed % 10;
  seconds = (elapsed % 100 - tenths) / 10;
  tens = (elapsed / 10 - seconds) / 10;
  
  if (tens <= 0) {
    drawNumber(5, 0, seconds, BRIGHTNESS, group->matrix);
    if (showDecimal) {
      group->matrix.drawPixel(10, 8, BRIGHTNESS);
      drawNumber(12, 0, tenths, BRIGHTNESS, group->matrix);
    }
  } else {
    drawNumber(0, 0, tens, BRIGHTNESS, group->matrix);
    drawNumber(5, 0, seconds, BRIGHTNESS, group->matrix);
    if (showDecimal){
      group->matrix.drawPixel(10, 8, BRIGHTNESS);
      drawNumber(12, 0, tenths, BRIGHTNESS, group->matrix);
    }
  }
}