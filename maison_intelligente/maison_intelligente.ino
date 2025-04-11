#include <LCD_I2C.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <HCSR04.h>

#define MOTOR_INTERFACE_TYPE 4

#define IN_1 31
#define IN_2 33
#define IN_3 35
#define IN_4 37

#define TRIGGER_PIN 9
#define ECHO_PIN 10

#define RED_PIN 3

#define BLUE_PIN 2
#define BUZZER 4

LCD_I2C lcd(0x27, 16, 2);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);

#define STEPS_PER_REVOLUTION 200
#define MIN_ANGLE 10
#define MAX_ANGLE 170
#define QUART_CIRCLE_STEPS_OPEN (STEPS_PER_REVOLUTION * (MAX_ANGLE - MIN_ANGLE) / 360.0)

unsigned long duration;
unsigned long distance;
String doorStatus = "Ferme";
unsigned long lastDistanceMeasurement = 0;

const unsigned long ALARM_THRESHOLD = 15;
const unsigned long LED_BLINK_INTERVAL = 500;
bool ledState = false;
unsigned long lastLedSwitchTime = 0;

bool isOpening = false;
bool isClosing = false;
int targetSteps = 0;
int currentDegree = MIN_ANGLE;

void setup() {
  lcd.begin();
  Serial.begin(9600);

  pinMode(RED_PIN, OUTPUT);

  pinMode(BLUE_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("2419796");
  lcd.setCursor(0, 1);
  lcd.print("Labo 5");
  delay(2000);

  lcd.clear();
  lcd.print("Distance:");
  lcd.setCursor(0, 1);
  lcd.print(doorStatus);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(500);
  myStepper.setSpeed(200);
}

unsigned long measureDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

void updateDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("Dist : ");
  lcd.print(distance);
  lcd.print(" cm    ");

  lcd.setCursor(0, 1);
  lcd.print("Porte : ");
  lcd.print(doorStatus);
}

unsigned long lastAlarmTime = 0;
bool alarmActive = false;

void handleAlarmEffects() {
  unsigned long currentTime = millis();

  if (distance <= ALARM_THRESHOLD) {

    digitalWrite(BUZZER, HIGH);
    alarmActive = true;
    lastAlarmTime = currentTime;

    if (currentTime - lastLedSwitchTime >= LED_BLINK_INTERVAL) {
      lastLedSwitchTime = currentTime;
      ledState = !ledState;
      digitalWrite(RED_PIN, ledState);
      digitalWrite(BLUE_PIN, !ledState);
    }

  } else {
    if (alarmActive && (currentTime - lastAlarmTime >= 3000)) {
      digitalWrite(BUZZER, LOW);
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BLUE_PIN, LOW);
      alarmActive = false;
    } else if (alarmActive) {
      if (currentTime - lastLedSwitchTime >= LED_BLINK_INTERVAL) {
        lastLedSwitchTime = currentTime;
        ledState = !ledState;
        digitalWrite(RED_PIN, ledState);
        digitalWrite(BLUE_PIN, !ledState);
      }
    }
  }
}

void openDoorNonBlocking() {
  if (!isOpening && doorStatus == "Ferme") {
    isOpening = true;
    isClosing = false;
    currentDegree = MIN_ANGLE;
    doorStatus = "Ouverture";
  }

  if (isOpening) {
    if (currentDegree <= MAX_ANGLE) {
      int steps = map(currentDegree, MIN_ANGLE, MAX_ANGLE, 0, QUART_CIRCLE_STEPS_OPEN);
      myStepper.moveTo(steps);
      myStepper.run();

      if (myStepper.distanceToGo() == 0) {
        currentDegree++;
      }

      lcd.setCursor(0, 1);
      lcd.print("Ouverture: ");
      lcd.print(currentDegree);
      lcd.print("dg    ");
    } else {
      isOpening = false;
      doorStatus = "Ouverte";
    }
  }
}

void closeDoorNonBlocking() {
  if (!isClosing && doorStatus == "Ouverte") {
    isClosing = true;
    isOpening = false;
    currentDegree = MAX_ANGLE;
    doorStatus = "Fermeture";
  }

  if (isClosing) {
    if (currentDegree >= MIN_ANGLE) {
      int steps = map(currentDegree, MIN_ANGLE, MAX_ANGLE, 0, QUART_CIRCLE_STEPS_OPEN);
      myStepper.moveTo(steps);
      myStepper.run();

      if (myStepper.distanceToGo() == 0) {
        currentDegree--;
      }

      lcd.setCursor(0, 1);
      lcd.print("Fermeture: ");
      lcd.print(currentDegree);
      lcd.print("dg    ");
    } else {
      isClosing = false;
      doorStatus = "Ferme";
    }
  }
}

void loop() {
  if (millis() - lastDistanceMeasurement >= 50) {
    lastDistanceMeasurement = millis();
    distance = measureDistance();
    updateDisplay();
  }

  handleAlarmEffects();

  if (distance < 60 && doorStatus == "Ferme" && !isOpening) {
    openDoorNonBlocking();
  } else if (distance > 60 && doorStatus == "Ouverte" && !isClosing) {
    closeDoorNonBlocking();
  }

  if (isOpening) openDoorNonBlocking();
  if (isClosing) closeDoorNonBlocking();
}