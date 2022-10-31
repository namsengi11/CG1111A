#include <MeMCore.h>

#define CONTROL_PIN_0 A2
#define CONTROL_PIN_1 A3

#define IR_RECEIVER_PIN 4
#define ULTRASONIC_PIN 12 // mCore port 1 -> digital pin 12
#define LDR_PIN A1

#define LDR_WAIT 10
#define LDR_INTERVAL 10 //in milliseconds'
#define LDR_TIMES 5

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2

MeLineFollower lineFinder(PORT_2);

#define TIMEOUT 30000 // in us
#define SPEED_OF_SOUND 340 // in m/s


bool DEBUG = true;

enum Component {
  R, G, B, IrEmmiter
};

bool shouldStop() {
  return lineFinder.readSensors() == S1_IN_S2_IN;
}

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
      if (DEBUG) {
        Serial.println("Enabled IR emmiter");
      }
      break;
    default: // default is R
      analogWrite(CONTROL_PIN_0, 255);
      analogWrite(CONTROL_PIN_1, 255);
      if (DEBUG) {
        Serial.println("Default: Enabled LED R");
      }
      break;
  }
}

int getLdrReading(int times) {
  delay(LDR_WAIT);   
  //find the average reading for the requested number of times of scanning LDR
  int total = 0;
  //take the reading as many times as requested and add them up
  for(int i = 0; i < times; i += 1){
     total += analogRead(LDR);
     delay(LDR_INTERVAL);
  }
  //calculate the average and return it
  return total/times;
}

float getDistanceIr () {
  
}

enum Color {}

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

void setup() {
  //ultrasonic initialization
  pinMode(ULTRASONIC_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_PIN, LOW);

  //control pins initialization
  enableComponent(R);

  Serial.begin(9600);

}

void loop() {
  delay(1000);
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

  delay(1000);
  enableComponent(R);
  delay(1000);
  enableComponent(G);
  delay(1000);
  enableComponent(B);

  delay(1000);
  // test line tracer
  Serial.println(shouldStop());
}
