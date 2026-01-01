/* Nested Balancing - Improved balancing algorithm for Zumo 32U4
 *
 * This sketch will contain an improved balancing program.
 * Currently set up with gyro calibration and hardware initialization.
 *
 * Hardware: Zumo 32U4 with LCD display
 */

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD display;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
Zumo32U4IMU imu;

// This is the average reading obtained from the gyro's Y axis
// during calibration.
float gyroOffsetY;

void setup()
{
  Wire.begin();

  // Set up the inertial sensors.
  imu.init();
  imu.enableDefault();
  imu.configureForBalancing();

  display.clear();
  display.print(F("Gyro cal"));
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Y axis reading to the total.
    gyroOffsetY += imu.g.y;
  }
  gyroOffsetY /= 1024;

  display.clear();
  display.print(F("Press A"));
  display.gotoXY(0, 1);
  display.print(F("to start"));
  ledYellow(0);

  // Wait for button A press to start balancing.
  while (!buttonA.getSingleDebouncedRelease()) {}

  display.clear();
  delay(500);
}

void loop()
{
  // Test motors with a 1-second counterclockwise turn
  motors.setSpeeds(-200, 200);  // Left backward, right forward = counterclockwise
  delay(1000);
  motors.setSpeeds(0, 0);       // Stop motors

  // Prevent repeating
  while (true) {}
}
