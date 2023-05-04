# PololuPID
This is an example code that demonstrates the use of the line sensors on the 3pi+ 32U4 to follow a black line on a white background, using a PID-based algorithm. The code uses the Pololu3piPlus32U4 library, which provides an interface to access the sensors and motors of the robot.

## Dependencies
This code requires the following libraries to be installed:
- Pololu3piPlus32U4
- PololuMenu
- Wire

## Functionality
The robot is capable of performing the following functions:
- Calibrate the line sensors and bump sensors
- Display distance traveled on the OLED screen
- Follow a black line on a white background using PID control

## Usage
To use this code, you will need to have a 3pi+ 32U4 robot from Pololu with the line sensors connected to the appropriate pins on the board.

Connect the robot to your computer, and upload the code using the Arduino IDE. Once the code is uploaded, the robot should be able to follow a black line on a white background. You can calibrate the sensors using the calibration function, and display the sensor readings using the sensor reading function.

## Credits

This code is based on the documentation provided by [Pololu](https://www.pololu.com/docs/0J83). 

## License
This code is released under the [MIT license](LICENSE).
