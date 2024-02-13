#include <Arduino.h>
#include <Servo.h>

#include <UTFT.h>
#include <memorysaver.h>
#include <AccelStepper.h>

const int SERVO_P2_PIN = 6;
const int SERVO_P3_PIN = 5;
const int SERVO_P4_PIN = 3;

const int PHOTO_S3_PIN = A0;

double stepsPerRevolution = 2048;

AccelStepper stepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11);

Servo servoP2;
Servo servoP3;
Servo servoP4;

int photoS3Values[50];
int photoS3ValuesIndex = 0;

// display :
extern uint8_t SmallFont[];

UTFT myGLCD(ST7735S_4L_80160, 12, 13, 7, 2, 4); // LCD:  4Line  serial interface      SDI  SCL  /CS  /RST  D/C    NOTE:Only support  DUE   MEGA  UNO
extern uint8_t BigFont[];
int color = 0;
int bsize = 4;
word colorlist[] = {VGA_GREEN, VGA_YELLOW, VGA_RED};
// end display

void setup()
{
  // Setup the servo's
  pinMode(SERVO_P2_PIN, OUTPUT);
  pinMode(SERVO_P3_PIN, OUTPUT);
  servoP2.attach(SERVO_P2_PIN);
  servoP2.write(0);
  servoP3.attach(SERVO_P3_PIN);
  servoP4.attach(SERVO_P4_PIN);

  // Setup the photoresistor
  pinMode(PHOTO_S3_PIN, INPUT);

  stepper.setMaxSpeed(500);
  stepper.setAcceleration(500);

  // Setup communication with the Pi
  Serial.begin(9600);
  myGLCD.InitLCD();
  myGLCD.setFont(BigFont);

  myGLCD.clrScr();
  Serial.println("Ready");
}

void loop()
{
  // Read the photoresistor
  int photoS3 = analogRead(PHOTO_S3_PIN);

  if (photoS3ValuesIndex > 49)
  {
    photoS3ValuesIndex = 0;
    int sum = 0;
    for (int i = 0; i < 50; i++)
    {
      sum += photoS3Values[i];
    }
    Serial.print("S3: ");
    Serial.println(photoS3);
  }
  photoS3Values[photoS3ValuesIndex] = photoS3;
  photoS3ValuesIndex++;

  while (Serial.available() > 0)
  {
    // Read the serial input
    char input = Serial.read();

    // Check if the input is P2
    if (input == 'B')
    {
      // Read the angle
      int angle = Serial.parseInt();
      Serial.print("P2: ");
      Serial.println(angle);

      // Set the angle of the servo
      servoP2.write(angle);
    }

    // Check if the input is P3
    else if (input == 'W')
    {
      // Read the angle
      int angle = Serial.parseInt();
      Serial.print("P3: ");
      Serial.println(angle);

      // Set the angle of the servo
      servoP3.write(angle);
    }

    // Check if the input is P4
    else if (input == 'S')
    {
      // Read the angle
      int steps = Serial.parseInt();
      Serial.print("P4: ");
      Serial.println(steps);

      // Set the angle of the servo
      stepper.moveTo(steps);
    }

    // Check if the input is P5
    else if (input == 'R')
    {
      // Read the angle
      int steps = Serial.parseInt();
      Serial.print("P5: ");
      Serial.println(steps);

      // Set the angle of the servo
      servoP4.write(steps);
    }

    // Check if the input is a delay program
    else if (input == 'D')
    {
      int delayAmount = Serial.parseInt();
      Serial.print("D: ");
      Serial.println(delayAmount);
      delay(delayAmount);
    }

    else if (input == 'E')
    {
      String errorCode = Serial.readString();
      int errorLevel = errorCode.toInt() / 100;
      word colorlist[] = {VGA_GREEN, VGA_YELLOW, VGA_RED};
      if (errorCode.toInt() == 0) {
        myGLCD.clrScr();
      }
      if (errorCode.toInt() == 1) {
        myGLCD.setFont(SmallFont);
        myGLCD.print("Starting...", 110, 60, 180);
        delay(3000);
        myGLCD.clrScr();
        myGLCD.print("Hello, group 22!", 140, 60, 180);
        delay(5000);
        myGLCD.clrScr();
        myGLCD.setFont(BigFont);
      }
      if (errorLevel < 4 && errorLevel > 0) {
        myGLCD.clrScr();
        myGLCD.setColor(colorlist[errorLevel - 1]);
        myGLCD.print("E"+errorCode, 120, 50, 180);
      }
    }
  }

  for (int i = 0; i < 100; i++)
  {
    if (stepper.distanceToGo() != 0)
    {
      stepper.run();
    }
  }

  // Wait for 0.01 second
  delay(1);
}