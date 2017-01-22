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

  //PID
  static constexpr float angle_p_default = .025;
  static constexpr float angle_i_default = .001;
  static constexpr float angle_d_default = .001;
  static constexpr float y_p_default = .025;
  static constexpr float y_i_default = .001;
  static constexpr float y_d_default = .001;
  static constexpr float x_p_default = .025;
  static constexpr float x_i_default = .001;
  static constexpr float x_d_default = .001;


  //Joystick Buttons
  static constexpr int runGearMoveThreadButton = 99;
  static constexpr int cancelGearMoveThreadButton = 99;4
  static constexpr int turnToGearButton = 1;
  static constexpr int moveToGearButton = 2;
  static constexpr int driveOneAxisButton = 3;
  static constexpr int gearReleaseButton = 99;

  //Joystick scaling constants
  const static constexpr float driveXDeadZone = .2;
  const static constexpr float driveXMax = 1;
  const static constexpr int driveXDegree = 1;
  const static constexpr float driveYDeadZone = .2;
  const static constexpr float driveYMax = 1;
  const static constexpr int driveYDegree = 1;
  const static constexpr float driveZDeadZone = .2;
  const static constexpr float driveZMax = .375;
  const static constexpr int driveZDegree = 1;

};
#endif
