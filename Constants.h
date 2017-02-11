#ifndef SRC_CONSTANTS_H
#define SRC_CONSTANTS_H

namespace Constants {

  //Pin Definitions
  static constexpr int frontLeftDriveChannel = 1;
  static constexpr int rearLeftDriveChannel = 4;
  static constexpr int frontRightDriveChannel = 2;
  static constexpr int rearRightDriveChannel = 3;
  static constexpr int driveStickChannel = 0;
  static constexpr int operatorStickChannel = 1;
  static constexpr int gearActuatorInSole = 4; //TODO: may change
  static constexpr int gearActuatorOutSole = 5; //TODO: may change
  static constexpr int rotatorChannel = 5;
  static constexpr int shooterChannel = 7;
  static constexpr int agitatorChannel = 0;
  static constexpr int compressorPin = 3;
  static constexpr int driveThrottleAxis = 3;
  static constexpr int intakeMotorPin = 99;
  static constexpr int verticalConveyorMotorPin = 1;
  static constexpr int climberPin = 6;
  static constexpr int brakesInSole = 6; //TODO: may change
  static constexpr int brakesOutSole = 7; //TODO: may change
  static constexpr int intakePDPChannel = 10;

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

  //Joystick
  static constexpr int moveToGearButton = 2;
  static constexpr int driveOneAxisButton = 7;
  static constexpr int gearActuateButton = 4;
  static constexpr int shooterAutoAngleButton = 5;
  static constexpr int shooterShootButton = 6;
  static constexpr int cancelAllButton = 3;
  static constexpr int driveXAxis = 0;
  static constexpr int driveYAxis = 1;
  static constexpr int driveZAxis = 2;
  static constexpr int fieldOrientedDriveButton = 14;
  static constexpr int swapCamerasButton = 8;
  static constexpr int intakeActivateButton = 1;
  static constexpr int climbButton = 13; //TODO: may want to change - ask drivers / kyle
  static constexpr int brakeButton = 11; //TODO: may want to change - ask drivers / kyle

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

  //Climber
  static constexpr int climberMaxSpeed = 512; //TODO: temp
  static constexpr int climberRunSpeed = 1.0;

  //Intake
  static constexpr float intakeRunSpeed = .5; //TODO: temp
  static constexpr float verticalConveyorRunSpeed = .5; //TODO: temp
  static constexpr float intakeCurrentMax = 100; //TODO: temp

};
#endif
