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
  static constexpr int gearActuatorInSole = 1; //TODO: may change
  static constexpr int gearActuatorOutSole = 2; //TODO: may change
  static constexpr int rotatorChannel = 5;
  static constexpr int shooterChannel = 7;
  static constexpr int agitatorChannel = 0;
  static constexpr int compressorPin = 3;
  static constexpr int driveThrottleAxis = 3;
  static constexpr int intakeMotorPin = 99;
  static constexpr int verticalConveyorMotorPin = 1;
  static constexpr int climberPin = 6;
  static constexpr int brakesInSole = 4; //TODO: may change
  static constexpr int brakesOutSole = 5; //TODO: may change
  static constexpr int intakePDPChannel = 10;
  static constexpr int xOmniEncoderPinA = 2;//0;
  static constexpr int xOmniEncoderPinB = 3;//1;
  static constexpr int yOmniEncoderPinA = 0;//2;
  static constexpr int yOmniEncoderPinB = 1;//3;
  static constexpr int leftIRPin = 4;
  static constexpr int rightIRPin = 5;
//  static constexpr int yEncoderPinA = 2;
//  static constexpr int yEncoderPinB = 3;
//  static constexpr int xEncoderPinA = 0;
//  static constexpr int xEncoderPinB = 1;
  static constexpr int leftProxPinA = 7;
  static constexpr int leftProxPinB = 6;
  static constexpr int rightProxPinA = 9;
  static constexpr int rightProxPinB = 8;
  static constexpr int gearPusherInSole = 4;
  static constexpr int gearPusherOutSole = 5;

  //PID
  static constexpr float angle_p_default = .008;
  static constexpr float angle_i_default = .0;
  static constexpr float angle_d_default = .0;
  static constexpr float y_p_default = .005;
  static constexpr float y_i_default = .001;
  static constexpr float y_d_default = .001;
  static constexpr float x_p_default = .013;
  static constexpr float x_i_default = .001;
  static constexpr float x_d_default = .001;
  static constexpr float angleErrorLimit = 1.0;

  //Joystick
  static constexpr int moveToGearButton = 2;
  static constexpr int driveOneAxisButton = 7;
  static constexpr int gearActuateButton = 13;
  static constexpr int shooterAutoAngleButton = 5;
  static constexpr int shooterShootButton = 6;
  static constexpr int cancelAllButton = 99;
  //static constexpr int driveXAxis = 2;
  //static constexpr int driveYAxis = 5;
  //static constexpr int driveZAxis = 0;
  static constexpr int driveXAxis = 0;
  static constexpr int driveYAxis = 1;
  static constexpr int driveZAxis = 2;
  static constexpr int fieldOrientedDriveButton = 99;
  static constexpr int swapCamerasButton = 8;
  static constexpr int swapDriveButton = 8;
  static constexpr int intakeActivateButton = 9;
  static constexpr int climbButton = 1; //TODO: may want to change - ask drivers / kyle
  static constexpr int climbDownButton = 12;
  static constexpr int brakeButton = 13; //TODO: may want to change - ask drivers / kyle
  static constexpr int agitateButton = 99;//10;
  static constexpr int reverseAgitatorButton = 11;
  static constexpr int climberSlowMoOperatorButton = 3; //operator stick
  static constexpr int climberSlowMoBackwardsOperatorButton = 4;
  static constexpr int driveFullPowerToggleButton = 4;
  static constexpr int climberMinPowerButton = 10;
  static constexpr int gearPusherButton = 14;
  static constexpr int gearDropButton = 3;

  //Button Defs
  /*
   * 1: Square
   * 2: X
   * 3: Circle
   * 4: Triangle
   * 5: L1
   * 6: R1
   * 7: L2
   * 8: R2
   * 9: Share
   * 10: Options
   * 11: Press Left Joystick
   * 12: Press Right Joystick
   * 13: PS4 Button
   * 14: Touchpad
   */

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
  static constexpr int shooterMaxSpeed = 4196; //TODO: temp
  static constexpr float autoShooterSpeed = .76;

  //Climber
  static constexpr int climberMaxSpeed = 512; //TODO: temp
  static constexpr int climberRunSpeed = 1.0;

  //Accumulator
  static constexpr float teleopLoopTime = 0.011;
  static constexpr float accumulatorPower = -0.01;
  static constexpr float yDistancePerSecond = 103.0;
  static constexpr float xDistancePerSecond = 36;
  static constexpr float rightCameraOffset = 7.5;
  static constexpr float leftCameraOffset = -7.5;
  static constexpr float minStrafePower = .35;
  static constexpr float minForwardPower = .1;
  static constexpr float minTurnPower = .2;
  static constexpr float accumulatorXp = .015;

  //Intake
  static constexpr float intakeRunSpeed = 1.0; //TODO: temp
  static constexpr float verticalConveyorRunSpeed = 1.0; //TODO: temp
  static constexpr float intakeCurrentMax = 100; //TODO: temp

};
#endif
