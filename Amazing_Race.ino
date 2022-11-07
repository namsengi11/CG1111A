#include <MeMCore.h>
#include <Math.h>

#define CONTROL_PIN_0 A2
#define CONTROL_PIN_1 A3
#define IR_RECEIVER_PIN A0
#define ULTRASONIC_PIN 12  // mCore port 1 -> digital pin 12
#define LDR_PIN A1

// LDR parameters
#define LDR_WAIT 40
#define LDR_INTERVAL 10  //in milliseconds
#define LDR_TIMES 5

// Ultrasonic sensor parameters
#define TIMEOUT 30000       // in us
#define SPEED_OF_SOUND 340  // in m/s
#define ultraSonicDistanceThreshold 9.5 // in cm

// IR parameter
#define irIntensityThreshold 65


/* Debug switch.
    0: no debug
    1: IR ,ultrasonic, line tracer and motors
    2: 1 and color sensor
    3: all

*/
int DEBUG = 0;

// Motors and parameters
MeDCMotor leftMotor(M1);   // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);  // assigning RightMotor to port M2

uint8_t rightSpeed = 255;
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
  White
};

// Always remeasure the rgb values below after changing this!
float whiteValues[] = { 704, 936, 801 };
float blackValues[] = { 288, 605, 431 };

// Always remeasure these values after changing the values above!
ColorRgb red = { 232, 118, 91 };
ColorRgb green = { 44, 181, 123 };
ColorRgb orange = { 250, 180, 135 };
ColorRgb purple = { 125, 158, 161 };
ColorRgb lightBlue = { 107, 218, 223 };
ColorRgb white = { 255, 255, 255 };

// To be initialized after startup
float greyDifference[] = { 0, 0, 0 };

// Direction enum
enum Direction {
  Left,
  Right
};

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
      if (DEBUG >= 3) {
        Serial.println("Enabled LED R");
      }
      break;
    case G:
      analogWrite(CONTROL_PIN_0, 255);
      analogWrite(CONTROL_PIN_1, 0);
      if (DEBUG >= 3) {
        Serial.println("Enabled LED G");
      }
      break;
    case B:
      analogWrite(CONTROL_PIN_0, 0);
      analogWrite(CONTROL_PIN_1, 255);

      if (DEBUG >= 3) {
        Serial.println("Enabled LED B");
      }
      break;
    case IrEmmiter:
      analogWrite(CONTROL_PIN_0, 0);
      analogWrite(CONTROL_PIN_1, 0);
      if (DEBUG >= 3) {
        Serial.println("Enabled IR emmiter");
      }
      break;
    default:  // default is R
      analogWrite(CONTROL_PIN_0, 255);
      analogWrite(CONTROL_PIN_1, 255);
      if (DEBUG >= 3) {
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

void startMovingForward() {
  leftMotor.run(-leftSpeed);
  rightMotor.run(rightSpeed);
}

void stopMoving() {
  leftMotor.run(0);
  rightMotor.run(0);
}

void adjust(Direction d) {
  switch (d) {
    case Left:
      leftMotor.run(-leftSpeed * 0.75);
      rightMotor.run(rightSpeed);
      break;
    case Right:
      leftMotor.run(-leftSpeed);
      rightMotor.run(rightSpeed * 0.75);
      break;
    default:
      if (DEBUG >= 1) {
        Serial.println("Invalid input for adjustment");
      }
  }
}

void turn(Direction d) {
  switch (d) {
    case Left:
      leftMotor.run(leftSpeed);
      rightMotor.run(rightSpeed);
      delay(345);
      break;
    case Right:
      leftMotor.run(-leftSpeed);
      rightMotor.run(-rightSpeed);
      delay(340);
      break;
    default:
      if (DEBUG >= 1) {
        Serial.println("Invalid input for turn");
        ;
      }
  }
  stopMoving();
}

void turn180() {
  leftMotor.run(-leftSpeed);
  rightMotor.run(-rightSpeed);
  delay(640);
  stopMoving();
}

void turnTwice(Direction d) {
  turn(d);
  startMovingForward();
  delay(900);
  stopMoving();
  delay(100);
  turn(d);
}

bool shouldStop() {
  bool isBlackStripDetected = lineFinder.readSensors() == S1_IN_S2_IN;
  if (DEBUG >= 1) {
    Serial.print("Detected black strip? ");
    Serial.println(isBlackStripDetected);
  }
  return isBlackStripDetected;
}

int getLdrReading(int times) {
  delay(LDR_WAIT);
  //find the average reading for the requested number of times of scanning LDR
  int total = 0;
  //take the reading as many times as requested and add them up
  for (int i = 0; i < times; i += 1) {
    int reading = analogRead(LDR_PIN);
    total += reading;
    if (DEBUG >= 4) {
      Serial.print("The ");
      Serial.print(i);
      Serial.print("th reading is ");
      Serial.println(reading);
    }
    delay(LDR_INTERVAL);
  }
  //calculate the average and return it
  int result = (int)(total / times);
  if (DEBUG >= 2) {
    Serial.print("LDR reading is ");
    Serial.println(result);
  }
  return result;
}

int getIntensityIr() {
  delay(10);
  enableComponent(IrEmmiter);
  delay(10);
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
    case White:
      if (DEBUG >= 2) {
        Serial.println("Detected white");
      }
      celebrate();
      delay(999999999);
      break;
    case Red:
      if (DEBUG >= 2) {
        Serial.println("Detected red");
      }
      turn(Left);
      break;
    case Green:
      if (DEBUG >= 2) {
        Serial.println("Detected green");
      }
      turn(Right);
      break;
    case Orange:
      if (DEBUG >= 2) {
        Serial.println("Detected orange");
      }
      turn180();
      break;
    case Purple:
      if (DEBUG >= 2) {
        Serial.println("Detected purple");
      }
      turnTwice(Left);
      break;
    case LightBlue:
      if (DEBUG >= 2) {
        Serial.println("Detected light blue");
      }
      turnTwice(Right);
      break;
    default:
      if (DEBUG >= 2) {
        Serial.println("No color detected, is there a bug?");
      }
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
  Color result = White;
  float minDiff = getColorDifference(c, white);
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
  if (DEBUG >= 2) {
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

  delay(2000);
  startMovingForward();
}

void loop() {
  if (shouldStop()) {
    stopMoving();
    if (DEBUG >= 1) {
      Serial.println("Stop!");
    }
    ColorRgb color = getColor();
    doAction(colorRgbToColor(color));
  } else {

    // test ir
    int intensity = getIntensityIr();
    if (DEBUG >= 1) {
      Serial.print("IR intensity is ");
      Serial.println(intensity);
    }

    // test ultrasonic
    float distance = getDistanceUltraSonic();
    if (distance > 0) {  // < 0 means reading not valid
      if (DEBUG >= 1) {
        Serial.print("Ultrasonic distance is ");
        Serial.println(distance);
      }
    } else {
      if (DEBUG >= 1) {
        Serial.println("Ultrasonic distance out of range!");
      }
    }

    // Do action according to readings
    if (distance < ultraSonicDistanceThreshold) {
      if (DEBUG >= 1) {
        Serial.println("Too left, turn right");
      }
      adjust(Right);
    } else if (intensity < irIntensityThreshold) {
      if (DEBUG >= 1) {
        Serial.println("Too right, turn left");
      }
      adjust(Left);
    } else {
      startMovingForward();
    }
  }
}
