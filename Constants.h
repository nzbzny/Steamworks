#ifndef SRC_CONSTANTS_H
#define SRC_CONSTANTS_H

namespace Constants {

  //Pin Definitions
  static constexpr int frontLeftDriveChannel = 2;
  static constexpr int rearLeftDriveChannel = 3;
  static constexpr int frontRightDriveChannel = 1;
  static constexpr int rearRightDriveChannel = 0;
  static constexpr int driveStickChannel = 0;
  static constexpr int operatorStickChannel = 1;
  static constexpr int gearReleaseInSole = 99;
  static constexpr int gearReleaseOutSole = 99;
  static constexpr int rotatorChannel = 99;
  static constexpr int shooterChannel = 99;
  static constexpr int agitatorChannel = 99;
  static constexpr int compressorPin = 99;
  static constexpr int driveXAxis = 0;
  static constexpr int driveYAxis = 1;
  static constexpr int driveZAxis = 2;
  static constexpr int driveThrottleAxis = 3;
  static constexpr int intakeMotorPin = 99;
  static constexpr int verticalConveyorMotorPin = 99;

  //PID
  static constexpr float angle_p_default = .025;
  static constexpr float angle_i_default = .001;
  static constexpr float angle_d_default = .001;
  static constexpr float y_p_default = .005;
  static constexpr float y_i_default = .001;
  static constexpr float y_d_default = .001;
  static constexpr float x_p_default = .013;
  static constexpr float x_i_default = .001;
  static constexpr float x_d_default = .001;


  //Joystick Buttons
  static constexpr int runGearMoveThreadButton = 99;
  static constexpr int cancelGearMoveThreadButton = 99;
  static constexpr int turnToGearButton = 1;
  static constexpr int moveToGearButton = 2;
  static constexpr int driveOneAxisButton = 3;
  static constexpr int gearReleaseButton = 99;
  static constexpr int shooterAutoAngleButton = 99;
  static constexpr int shooterShootButton = 99;
  static constexpr int cancelAllButton = 99;

  //Joystick scaling constants
  static constexpr float driveXDeadZone = .2;
  static constexpr float driveXMax = 1;
  static constexpr int driveXDegree = 1;
  static constexpr float driveYDeadZone = .2;
  static constexpr float driveYMax = 1;
  static constexpr int driveYDegree = 1;
  static constexpr float driveZDeadZone = .2;
  static constexpr float driveZMax = .375;
  static constexpr int driveZDegree = 1;

  //Shooter
  static constexpr int shooterMaxSpeed = 512; //TODO: temp

};
#endif
