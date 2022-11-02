#include <MeMCore.h>
#include <Math.h>

#define CONTROL_PIN_0 A2
#define CONTROL_PIN_1 A3
#define IR_RECEIVER_PIN A0
#define ULTRASONIC_PIN 12  // mCore port 1 -> digital pin 12
#define LDR_PIN A1

// LDR parameters
#define LDR_WAIT 20
#define LDR_INTERVAL 10  //in milliseconds'
#define LDR_TIMES 5

// Ultrasonic sensor parameters
#define TIMEOUT 30000       // in us
#define SPEED_OF_SOUND 340  // in m/s

// Debug switch
bool DEBUG = true;

// Motors and parameters
MeDCMotor leftMotor(M1);   // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);  // assigning RightMotor to port M2

uint8_t rightSpeed = 212;
uint8_t leftSpeed = 255;

// Line follower
MeLineFollower lineFinder(PORT_2);

// Buzzer
MeBuzzer buzzer;  // create the buzzer object

// ColorRgb struct and Color enum definition
typedef struct ColorsRgb {
  int r;
  int g;
  int b;
} ColorRgb;

enum Color {
  Red,
  Green,
  Orange,
  Purple,
  LightBlue,
  Black
};

//Zhengdao's values at home
float whiteValues[] = { 741, 895, 813 };
float blackValues[] = { 391, 530, 559 };
ColorRgb red = { 244, 129, 115 };
ColorRgb green = { 24, 150, 110 };
ColorRgb orange = { 255, 170, 143 };
ColorRgb purple = { 129, 144, 148 };
ColorRgb lightBlue = { 95, 198, 200 };
ColorRgb black = { 0, 0, 0 };

/*
float whiteValues[] = {813, 921, 843};
float blackValues[] = {500, 612, 633};
ColorRgb red = {244, 129, 115};
ColorRgb green = {24, 150, 110};
ColorRgb orange = {255, 170, 143};
ColorRgb purple = {129, 144, 148};
ColorRgb lightBlue = {95, 198, 200};
ColorRgb black = {0, 0, 0};
*/

// To be initialized after startup
float greyDifference[] = { 0, 0, 0 };


// Component management
enum Component {
  R,
  G,
  B,
  IrEmmiter
};

void enableComponent(Component c) {
  switch (c) {
    case R:
      analogWrite(CONTROL_PIN_0, 255);
      analogWrite(CONTROL_PIN_1, 255);
      if (DEBUG) {
        Serial.println("Enabled LED R");
      }
      break;
    case G:
      analogWrite(CONTROL_PIN_0, 255);
      analogWrite(CONTROL_PIN_1, 0);
      if (DEBUG) {
        Serial.println("Enabled LED G");
      }
      break;
    case B:
      analogWrite(CONTROL_PIN_0, 0);
      analogWrite(CONTROL_PIN_1, 255);

      if (DEBUG) {
        Serial.println("Enabled LED B");
      }
      break;
    case IrEmmiter:
      analogWrite(CONTROL_PIN_0, 0);
      analogWrite(CONTROL_PIN_1, 0);
      if (DEBUG) {
        Serial.println("Enabled IR emmiter");
      }
      break;
    default:  // default is R
      analogWrite(CONTROL_PIN_0, 255);
      analogWrite(CONTROL_PIN_1, 255);
      if (DEBUG) {
        Serial.println("Default: Enabled LED R");
      }
      break;
  }
}

// Course logic and utils
void celebrate() {
  // Each of the following "function calls" plays a single tone.
  // The numbers in the bracket specify the frequency and the duration (ms)
  buzzer.tone(392, 200);
  buzzer.tone(523, 200);
  buzzer.tone(659, 200);
  buzzer.tone(784, 400);
  buzzer.tone(659, 200);
  buzzer.tone(784, 600);
  buzzer.noTone();
}

void turn(int direction) {  // for direction, 0 is left, 1 is right
  if (direction == 0) {
    leftMotor.run(leftSpeed);
    rightMotor.run(rightSpeed);
    delay(345);
  } else {
    leftMotor.run(-leftSpeed);
    rightMotor.run(-rightSpeed);
    delay(340);
  }
  leftMotor.run(0);
  rightMotor.run(0);
}

void turn180() {
  leftMotor.run(leftSpeed);
  rightMotor.run(rightSpeed);
  delay(640);
  leftMotor.run(0);
  rightMotor.run(0);
}

void turnTwice(int direction) {
  turn(direction);
  leftMotor.run(-leftSpeed);
  rightMotor.run(rightSpeed);
  delay(500);
  leftMotor.run(0);
  rightMotor.run(0);
  delay(100);
  turn(direction);
}

void adjust(int i) { // i = 0 left, i = 1 right
  if (i == 0) {
    Serial.println("adjusting left");
//    while (getIntensityIr < 550) { // adjusting left uses ir
//      leftMotor.run(-255);
//      rightMotor.run(150);
//    }
  } else {
    Serial.println("adjusting right");
//    while (getDistanceUltraSonic < 5.5) { // adjusting right uses ultrasonic
//      leftMotor.run(-150);
//      rightMotor.run(212);
//    }
  }
}

bool shouldStop() {
  return lineFinder.readSensors() == S1_IN_S2_IN;
}

int getLdrReading(int times) {
  delay(LDR_WAIT);
  //find the average reading for the requested number of times of scanning LDR
  int total = 0;
  //take the reading as many times as requested and add them up
  for (int i = 0; i < times; i += 1) {
    int reading = analogRead(LDR_PIN);
    total += reading;
    /*
    if (DEBUG) {
      Serial.print("The ");
      Serial.print(i);
      Serial.print("th reading is ");
      Serial.println(reading);
    }
    */
    delay(LDR_INTERVAL);
  }
  //calculate the average and return it
  int result = (int)(total / times);
  if (DEBUG) {
    Serial.print("LDR reading is ");
    Serial.println(result);
  }
  return result;
}

int getIntensityIr() {
  delayMicroseconds(2);
  enableComponent(IrEmmiter);
  delayMicroseconds(10);
  int intensity = analogRead(IR_RECEIVER_PIN);
  enableComponent(R);
  return intensity;
}

float getDistanceUltraSonic() {
  pinMode(ULTRASONIC_PIN, OUTPUT);

  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_PIN, LOW);

  pinMode(ULTRASONIC_PIN, INPUT);
  long duration = pulseIn(ULTRASONIC_PIN, HIGH, TIMEOUT);
  if (duration > 0) {
    return ((float)duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100);
  } else {
    return (-1);
  }
}

void doAction(Color c) {
  switch (c) {
    case Black:
      Serial.println("Detected black");
      break;
    case Red:
      Serial.println("Detected red");
      break;
    case Green:
      Serial.println("Detected green");
      break;
    case Orange:
      Serial.println("Detected orange");
      break;
    case Purple:
      Serial.println("Detected purple");
      break;
    case LightBlue:
      Serial.println("Detected LightBlue");
      break;
    default:
      Serial.println("No color detected, is there a bug?");
  }
}


// Color related stuff

float normalizeProportion(float n) {
  float result = n;
  if (n < 0) {
    result = 0;
  } else if (n > 1) {
    result = 1;
  }
  // Squaring result to produce a y = x ^ 2 curve.
  //result = result * result;
  return result;
}

float getColorDifference(ColorRgb colorA, ColorRgb colorB) {
  //quadratic approach
  return pow(colorA.r - colorB.r, 2) + pow(colorA.g - colorB.g, 2) + pow(colorA.b - colorB.b, 2);
  //linear approach
  //return fabs(colorA.r - colorB.r) + fabs(colorA.g - colorB.g) + fabs(colorA.b - colorB.b);
}

Color colorRgbToColor(ColorRgb c) {
  Color result = Black;
  float minDiff = getColorDifference(c, black);
  float currentDiff = getColorDifference(c, red);
  if (currentDiff < minDiff) {
    result = Red;
    minDiff = currentDiff;
  }
  currentDiff = getColorDifference(c, red);
  if (currentDiff < minDiff) {
    result = Red;
    minDiff = currentDiff;
  }
  currentDiff = getColorDifference(c, green);
  if (currentDiff < minDiff) {
    result = Green;
    minDiff = currentDiff;
  }
  currentDiff = getColorDifference(c, orange);
  if (currentDiff < minDiff) {
    result = Orange;
    minDiff = currentDiff;
  }
  currentDiff = getColorDifference(c, purple);
  if (currentDiff < minDiff) {
    result = Purple;
    minDiff = currentDiff;
  }
  currentDiff = getColorDifference(c, lightBlue);
  if (currentDiff < minDiff) {
    result = LightBlue;
  }
  return result;
}

ColorRgb getColor() {
  ColorRgb color;
  enableComponent(R);
  float rProportion = ((float)(getLdrReading(LDR_TIMES)) - blackValues[0]) / (greyDifference[0]);
  rProportion = normalizeProportion(rProportion);
  color.r = rProportion * 255;
  enableComponent(G);
  float gProportion = ((float)(getLdrReading(LDR_TIMES)) - blackValues[1]) / (greyDifference[1]);
  gProportion = normalizeProportion(gProportion);
  color.g = gProportion * 255;
  enableComponent(B);
  float bProportion = ((float)(getLdrReading(LDR_TIMES)) - blackValues[2]) / (greyDifference[2]);
  bProportion = normalizeProportion(bProportion);
  color.b = bProportion * 255;
  enableComponent(R);  // change to default state
  if (DEBUG) {
    Serial.print("Normalized R value is ");
    Serial.println(color.r);
    Serial.print("Normalized G value is ");
    Serial.println(color.g);
    Serial.print("Normalized B value is ");
    Serial.println(color.b);
  }
  return color;
}

void setup() {
  //ultrasonic initialization
  pinMode(ULTRASONIC_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_PIN, LOW);

  //IR initialization
  pinMode(IR_RECEIVER_PIN, INPUT);

  //control pins initialization
  enableComponent(R);

  //initialize grey difference
  for (int i = 0; i < 3; i += 1) {
    greyDifference[i] = whiteValues[i] - blackValues[i];
  }

  Serial.begin(9600);
}

void loop() {
  // test ultrasonic
  float distance = getDistanceUltraSonic();
  if (distance > 0) {
    if (DEBUG) {
      Serial.print("Distance is ");
      Serial.println(distance);
    }
  } else {
    if (DEBUG) {
      Serial.println("Out of range!");
    }
  }

  delay(500);

  int intensity = getIntensityIr();
  if (DEBUG) {
    Serial.print("Intensity is ");
    Serial.println(intensity);
  }
  delay(500);

  ColorRgb color = getColor();
  doAction(colorRgbToColor(color));

  delay(500);
  // test line tracer
  Serial.println(shouldStop());
  delay(500);

  celebrate();
  delay(500);
  if (getIntensityIr() < 550) {
    adjust(0);
  } else if (getDistanceUltraSonic < 5.5) {
    adjust(1);
  }
}
