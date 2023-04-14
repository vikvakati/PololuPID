/* This example uses the line sensors on the 3pi+ 32U4 to follow
a black line on a white background, using a PID-based algorithm.*/

#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>
#include <Wire.h>

using namespace Pololu3piPlus32U4;

OLED display;
BumpSensors bumpSensors;
LineSensors lineSensors;
Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;

int16_t lastError = 0;
unsigned long start_time;  // to hold the start time of the movement
unsigned long elapsed_time; // to hold the elapsed time of the movement


#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];


// Maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 would let the motors go at top speed
uint16_t maxSpeed;
int16_t minSpeed;
int16_t turnTime;

// Speed the motors will run when centered on the line.
uint16_t baseSpeed;

uint16_t calibrationSpeed;

// PID configuration:
// proportional constant of 0.35 and a derivative constant of 0.35.
uint16_t proportional; 
uint16_t derivative;

// monitor distance traveled by each wheel
const char encoderErrorLeft[] PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";
char report[80];

void selectTurtle()
{
  maxSpeed = 350;
  minSpeed = 0;
  baseSpeed = maxSpeed;
  calibrationSpeed = 50;
  proportional = 0.35; // P coefficient = 0.35
  derivative = 0.35;   // D coefficient = 0.35
  turnTime = 450;
}

PololuMenu<typeof(display)> menu;

// Sets up special characters to display bar graphs.
void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  display.loadCustomCharacter(levels + 0, 0);  // 1 bar
  display.loadCustomCharacter(levels + 1, 1);  // 2 bars
  display.loadCustomCharacter(levels + 2, 2);  // 3 bars
  display.loadCustomCharacter(levels + 3, 3);  // 4 bars
  display.loadCustomCharacter(levels + 4, 4);  // 5 bars
  display.loadCustomCharacter(levels + 5, 5);  // 6 bars
  display.loadCustomCharacter(levels + 6, 6);  // 7 bars
}

void printBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, (char)255};
  display.print(barChars[height]);
}

void calibrateSensors()
{
  display.clear();

  bumpSensors.calibrate();

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for(uint16_t i = 0; i < 80; i++)
  {
    if (i > 20 && i <= 60)
    {
      motors.setSpeeds(-(int16_t)calibrationSpeed, calibrationSpeed);
    }
    else
    {
      motors.setSpeeds(calibrationSpeed, -(int16_t)calibrationSpeed);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

// Displays the estimated line position and a bar graph of sensor
// readings on the LCD. Returns after the user presses B.
void showReadings()
{
  display.clear();

  while(!buttonB.getSingleDebouncedPress())
  {
    uint16_t position = lineSensors.readLineBlack(lineSensorValues);

    display.gotoXY(0, 0);
    display.print(position);
    display.print("    ");
    display.gotoXY(0, 1);
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
      uint8_t barHeight = map(lineSensorValues[i], 0, 1000, 0, 8);
      printBar(barHeight);
    }
    delay(50);
  }
}

// follow path using line sensors and PID control
void followLine()
{
  // Get the position of the line. 
  int16_t position = lineSensors.readLineBlack(lineSensorValues);
  int16_t totalError = 0;

  // Check if all line sensors are off (no line detected)
  if (position == 0) {
    delay(50);
    // Stop the motors
    motors.setSpeeds(0, 0);
    elapsed_time = millis() - start_time;
    display.gotoXY(0, 1);
    display.print(elapsed_time/1000);
    display.print("s");
  }

  // Error is how far away from the center of the
  // line, which corresponds to position 2000.
  int16_t error = position - 2000;

  // Get motor speed difference using proportional and derivative
  // PID terms (the integral term is generally not very useful
  // for line following).
  int16_t speedDifference = error * (int32_t)proportional  + (error - lastError) * (int32_t)derivative;

  lastError = error;
  
  totalError += lastError;

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int16_t leftSpeed = (int16_t)baseSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)baseSpeed - speedDifference;

  // Constrain motor speeds to be between 0 and maxSpeed.
  // One motor will always be turning at maxSpeed, and the other
  // will be at maxSpeed-|speedDifference| if that is positive,
  // else it will be stationary.
  leftSpeed = constrain(leftSpeed, minSpeed, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, (int16_t)maxSpeed);

  motors.setSpeeds(leftSpeed, rightSpeed);
}

// turn around if bump sensor is pressed
void pressBump()
{
   bumpSensors.read();

  if (bumpSensors.leftIsPressed())
  {
    // Left bump sensor is pressed.
    ledYellow(true);
    motors.setSpeeds(0, 0);
    buzzer.play("a32");
    display.gotoXY(0, 0);
    display.clear();
    display.print('L');

    motors.setSpeeds(maxSpeed, -maxSpeed);
    delay(turnTime);

    motors.setSpeeds(0, 0);
    buzzer.play("b32");
    ledYellow(false);
    display.gotoXY(0, 0);
    display.clear();
  }
  else if (bumpSensors.rightIsPressed())
  {
    // Right bump sensor is pressed.
    ledRed(true);
    motors.setSpeeds(0, 0);
    buzzer.play("e32");
    display.gotoXY(7, 0);
    display.clear();
    display.print('R');

    motors.setSpeeds(-maxSpeed, maxSpeed);
    delay(turnTime);

    motors.setSpeeds(0, 0);
    buzzer.play("f32");
    ledRed(false);
    display.gotoXY(7, 0);
    display.clear();
  }
}

// display distance traveled from encoders
void calculateDistance()
{
  static uint8_t lastDisplayTime;
  static uint8_t displayErrorLeftCountdown = 0;
  static uint8_t displayErrorRightCountdown = 0;

  if ((uint8_t)(millis() - lastDisplayTime) >= 100)
  {
    lastDisplayTime = millis();

    int16_t countsLeft = encoders.getCountsLeft();
    int16_t countsRight = encoders.getCountsRight();

    bool errorLeft = encoders.checkErrorLeft();
    bool errorRight = encoders.checkErrorRight();

    if (errorLeft)
    {
      // An error occurred on the left encoder channel.
      // Display it for the next 10 iterations and also beep.
      displayErrorLeftCountdown = 10;
      buzzer.playFromProgramSpace(encoderErrorLeft);
    }

    if (errorRight)
    {
      // An error occurred on the right encoder channel.
      // Display for the next 10 iterations and also beep.
      displayErrorRightCountdown = 10;
      buzzer.playFromProgramSpace(encoderErrorRight);
    }

    // Convert encoder counts to distance in cm.
    // distance = (counts / 120) * (pi * wheel_diameter / 10)
    float distanceLeft = countsLeft * 20.4 / 4096;
    float distanceRight = countsRight * 20.4 / 4096;
    float distance = (distanceLeft + distanceRight) / 2;

    // Update the screen with encoder counts and error info.
    display.noAutoDisplay();
    display.clear();
    display.print(distance);
    display.print("cm");
    if (displayErrorLeftCountdown)
    {
      // Show an exclamation point on the first line to
      // indicate an error from the left encoder.
      display.gotoXY(7, 0);
      display.print('!');
      displayErrorLeftCountdown--;
    }
    if (displayErrorRightCountdown)
    {
      // Show an exclamation point on the second line to
      // indicate an error from the left encoder.
      display.gotoXY(7, 1);
      display.print('!');
      displayErrorRightCountdown--;
    }
    display.display();

    // Send the information to the serial monitor also.
    snprintf_P(report, sizeof(report),
        PSTR("%6d %6d %1d %1d"),
        countsLeft, countsRight, errorLeft, errorRight);
    Serial.println(report);
  }
}


void setup()
{
  loadCustomCharacters();

  // Play a welcome song
  buzzer.play(">g32>>c32");

  selectTurtle();

  // Wait for button B to be pressed and released.
  display.clear();
  display.print(F("Press B"));
  display.gotoXY(0, 1);
  display.print(F("to calib"));
  while(!buttonB.getSingleDebouncedPress());

  calibrateSensors();

  showReadings();

  // Play music and wait for it to finish before driving.
  display.clear();
  display.print(F("Go!"));
  buzzer.play("L16 cdegreg4");
  while(buzzer.isPlaying());
  
  // initialize the start time
  start_time = millis();
}

void loop()
{
  followLine();

  pressBump();

  calculateDistance();
}
