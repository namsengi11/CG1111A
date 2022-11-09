#include <MeMCore.h>
#include <Math.h>

#define CONTROL_PIN_0 A2
#define CONTROL_PIN_1 A3
#define IR_RECEIVER_PIN A0
#define ULTRASONIC_PIN 12
#define LDR_PIN A1

// LDR parameters
#define LDR_WAIT 40 // Time to wait before LDR takes the first reading, after LED is enabled. In ms
#define LDR_INTERVAL 10  // Time to wait between each LDR reading. In ms
#define LDR_TIMES 5 // Number of LDR readings to take for each color.

// Ultrasonic sensor parameters
#define TIMEOUT 30000       // Timeout for ultrasonic sensor. Any reading larger than this will be ignored. In us
#define SPEED_OF_SOUND 340  // Speed of sound. In m/s
#define ultraSonicDistanceThreshold 9.5 // Robot will turn if distance is below this threshold. In cm

/*  IR parameter
 *  Due to the way in which the circuit is constructed, the following reading decreases if IR intensity increases.
 *  Hence it is not actually IR intensity, but just the analog reading from the port.
 */
#define irIntensityThreshold 65 // Robot will turn if intensity falls below this threshold.


/*  Debug switch.
 *  0: No debug. Use for actual runs
 *  1: IR ,ultrasonic, line tracer and motors
 *  2: 1 and color sensor
 *  3: All except detailed LDR readings
 *  4: All
 */
int DEBUG = 0;

MeDCMotor leftMotor(M1);   // Assigning leftMotor to port M1
MeDCMotor rightMotor(M2);  // Assigning RightMotor to port M2

// Motor speeds for going straight.
uint8_t rightSpeed = 255;
uint8_t leftSpeed = 255;

// Line follower port assignment
MeLineFollower lineFinder(PORT_2);

// Buzzer 
MeBuzzer buzzer;  // Create the buzzer object

// ColorRgb struct and Color enum definition
typedef struct ColorsRgb {
  int r;
  int g;
  int b;
} ColorRgb;

// Only contains the colors of color paper
enum Color {
  Red,
  Green,
  Orange,
  Purple,
  LightBlue,
  White
};

// LDR readings for black and white color
// Always remeasure the rgb values below after changing this! This affects the RGB values obtained.
float whiteValues[] = { 704, 936, 801 };
float blackValues[] = { 288, 605, 431 };

// RGB values for color paper. Range is from 0~255.
// Always remeasure these values after changing the values above! Changes in the values above affects the RGB values obtained.
ColorRgb red = { 232, 118, 91 };
ColorRgb green = { 44, 181, 123 };
ColorRgb orange = { 250, 180, 135 };
ColorRgb purple = { 125, 158, 161 };
ColorRgb lightBlue = { 107, 218, 223 };
ColorRgb white = { 255, 255, 255 };

// To be initialized after startup. Grey difference = white value - black value.
float greyDifference[] = { 0, 0, 0 };

// Direction enum
enum Direction {
  Left,
  Right
};

// Enum for component management
enum Component {
  R,
  G,
  B,
  IrEmmiter
};

//Enables a given component. The component will remain on after returning from this function.
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

// Following functions are for course logic and utilities


// Plays the celebration song
void celebrate() {
  // Each of the following "function calls" plays a single tone.
  // The numbers in the bracket specify the frequency and the duration (in ms)
  buzzer.tone(392, 200);
  buzzer.tone(523, 200);
  buzzer.tone(659, 200);
  buzzer.tone(784, 400);
  buzzer.tone(659, 200);
  buzzer.tone(784, 600);
  buzzer.noTone();
}

// Makes the robot move forward. The robot will continue to move after returning from this function.
void startMovingForward() {
  leftMotor.run(-leftSpeed);
  rightMotor.run(rightSpeed);
}

// Stops the robot from moving.
void stopMoving() {
  leftMotor.run(0);
  rightMotor.run(0);
}

// Adjust the direction of the robot according to readings from IR receiver and ultrasonic sensor.
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

// Makes the robot do a 90 degree turn. The robot will stop after returning from this function.
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

// Makes the robot do a clockwise 180 degree turn. The robot will stop after returning from this function.
void turn180() {
  leftMotor.run(-leftSpeed);
  rightMotor.run(-rightSpeed);
  delay(640);
  stopMoving();
}

// Makes the robot do a 90 degree turn, move forward for one chunk, and do another 90 degree turn in the same direction.
// The robot will stop after returning from this function.
void turnTwice(Direction d) {
  turn(d);
  startMovingForward();
  delay(900);
  stopMoving();
  delay(100);
  turn(d);
}

// Ruturns true if black strip is detected. False otherwise.
bool shouldStop() {
  bool isBlackStripDetected = lineFinder.readSensors() == S1_IN_S2_IN;
  if (DEBUG >= 1) {
    Serial.print("Detected black strip? ");
    Serial.println(isBlackStripDetected);
  }
  return isBlackStripDetected;
}

// Takes a given number of times of LDR readings, and returns the average reading.
int getLdrReading(int times) {
  delay(LDR_WAIT);
  int total = 0;
  // Take the reading as many times as requested and add them up
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
  // Calculates the average and return it
  int result = (int)(total / times);
  if (DEBUG >= 2) {
    Serial.print("LDR reading is ");
    Serial.println(result);
  }
  return result;
}

// Returns the analog reading from IR receiver. The return value decreases if IR intensity increases.
int getIntensityIr() {
  delay(10);
  enableComponent(IrEmmiter);
  delay(10);
  int intensity = analogRead(IR_RECEIVER_PIN);
  enableComponent(R);
  return intensity;
}

// Returns the distance to the wall measured by the ultrasonic sensor.
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

// Makes the robot do action according to the given color.
void doAction(Color c) {
  switch (c) {
    case White:
      if (DEBUG >= 2) {
        Serial.println("Detected white");
      }
      celebrate();
      delay(999999999); // Stops the robot
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


// Following functions are for color-related jobs.


// Normalizes a ratio value. The return value will be between 0 ~ 1.
float normalizeProportion(float n) {
  float result = n;
  if (n < 0) {
    result = 0;
  } else if (n > 1) {
    result = 1;
  }
  // Squaring result to produce a y = x ^ 2 curve. Not as good as the linear approach, hence not used.
  //result = result * result;
  return result;
}


// Returns the difference between two colors given their respective RGB values.
float getColorDifference(ColorRgb colorA, ColorRgb colorB) {
  // Quadratic approach.
  return pow(colorA.r - colorB.r, 2) + pow(colorA.g - colorB.g, 2) + pow(colorA.b - colorB.b, 2);
  // Linear approach. Not as good as the quadratic approach, hence not used.
  //return fabs(colorA.r - colorB.r) + fabs(colorA.g - colorB.g) + fabs(colorA.b - colorB.b);
}

// Converts a ColorRgb to a Color enum.
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

// Gets the ColorRgb detected using diodes and LDR.
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

  //Control pins initialization. Turn on the red LED since it is the default state.
  enableComponent(R);

  //Initialize grey differences
  for (int i = 0; i < 3; i += 1) {
    greyDifference[i] = whiteValues[i] - blackValues[i];
  }
  Serial.begin(9600);

  // Wait for 2 seconds
  delay(2000);

  // Make the robot move forward.
  startMovingForward();
}

void loop() {
  // Checks if black strip is detected.
  if (shouldStop()) { // Black strip detected. Do action according to color detected.
    stopMoving();
    if (DEBUG >= 1) {
      Serial.println("Stop!");
    }
    ColorRgb color = getColor();
    doAction(colorRgbToColor(color));
  } else { // Black strip not detected. Adjust the direction of the robot according to IR and ultrasonic readings.
    // Obtain IR reading.
    int intensity = getIntensityIr();
    if (DEBUG >= 1) {
      Serial.print("IR intensity is ");
      Serial.println(intensity);
    }

    // Obtain ultrasonic reading.
    float distance = getDistanceUltraSonic();
    if (distance > 0) {
      if (DEBUG >= 1) {
        Serial.print("Ultrasonic distance is ");
        Serial.println(distance);
      }
    } else { // Reading < 0 means reading not valid
      if (DEBUG >= 1) {
        Serial.println("Ultrasonic distance out of range!");
      }
    }

    // Adjust the direction according to the readings.
    // The ultrasonic sensor has a higher priority, because its reading is more stable and reliable than the IR receiver.
    if (distance < ultraSonicDistanceThreshold) { // Too close to left walls, adjust to right.
      if (DEBUG >= 1) {
        Serial.println("Too left, turn right");
      }
      adjust(Right);
    } else if (intensity < irIntensityThreshold) { // Too close to right walls, adjust to left.
      if (DEBUG >= 1) {
        Serial.println("Too right, turn left");
      }
      adjust(Left);
    } else { // Not too close to any walls. Move forward.
      startMovingForward();
    }
  }
}
