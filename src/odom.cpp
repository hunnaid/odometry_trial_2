#include "vex.h"
#include "odom.h"
#include "robot-config.h"

using namespace vex;

// constants
const double wheelDiameter = 2.75; // inches
const double wheelCircumference = 3.141592653589793 * wheelDiameter;

// GLOBAL COORDINATES
double Odom::globalX = 0.0;
double Odom::globalY = 0.0;
double Odom::globalAngle = 0.0;
double Odom::prevGlobalX = 0.0;
double Odom::prevGlobalY = 0.0;

// LOCAL COORDINATES
Point Odom::localDeltaPoint = {0, 0};

// SENSOR VALUES
// motor values
double Odom::xEncoderPos = xEncoder.rotation(degrees);
double Odom::yEncoderPos = yEncoder.rotation(degrees);// Separate right back wheel motor
// angle
double Odom::currentAngle = imu.heading(degrees);
double Odom::prevAngle = 0.0;

double Odom::prevXEncoderPos = 0.0;
double Odom::prevYEncoderPos = 0.0;

double Odom::deltaAngle = currentAngle - prevAngle;
// ODOMETRY FUNCTIONS
void Odom::updateSensors() {
  xEncoderPos = xEncoder.rotation(degrees);
  yEncoderPos = yEncoder.rotation(degrees);
  // Replace encoder values with motor values
  double xEncoderDelta = xEncoderPos - prevXEncoderPos;
  double yEncoderDelta = yEncoderPos - prevYEncoderPos;

  // Update angle
  deltaAngle = currentAngle - prevAngle;
  prevAngle = currentAngle;

  // Update motor values
  double xDelta = xEncoderDelta * (wheelCircumference / 360.0);
  double yDelta = yEncoderDelta * (wheelCircumference / 360.0);

  // Polar coordinates
  localDeltaPoint.x = (yDelta + xDelta) / 2.0;
  localDeltaPoint.y = (yDelta - xDelta) / 2.0;

  // Cartesian coordinates
  // globalX += (localDeltaPoint.y * sin(prevAngle + deltaAngle / 2)) + (localDeltaPoint.x * cos(prevAngle + deltaAngle / 2));
  // globalY += (localDeltaPoint.y * cos(prevAngle + deltaAngle / 2)) - (localDeltaPoint.x * sin(prevAngle + deltaAngle / 2));
  globalX += localDeltaPoint.x * cos(globalAngle) - localDeltaPoint.y * sin(globalAngle);
  globalY += localDeltaPoint.x * sin(globalAngle) + localDeltaPoint.y * cos(globalAngle);

  globalAngle = currentAngle;

  // Update previous motor values
  prevXEncoderPos = xEncoderPos;
  prevYEncoderPos = yEncoderPos;
}

void Odom::reset() {
  xEncoderPos = 0.0;
  yEncoderPos = 0.0;
  prevAngle = globalAngle;
  prevGlobalX = globalX;
  prevGlobalY = globalY;
}

void Odom::setPosition(double newX, double newY, double newAngle) {
  reset();
  prevAngle = newAngle;
  prevGlobalX = newX;
  prevGlobalY = newY;
}

// ODOMETRY THREAD
int Odom::Odometry() {
  while (true) {

    Odom::updateSensors();
    // Print or use globalX, globalY, globalAngle as needed

    this_thread::sleep_for(10);
  }
  return 0;
}


// clamp fuck

/*
double Odom::clamp(double value, double min=-12.0, double max=12.0) {
  // Ensure value is not less than min
  if (value < min) {
      value = min;
  }

  // Ensure value is not greater than max
  if (value > max) {
      value = max;
  }

  return value;
}
*/