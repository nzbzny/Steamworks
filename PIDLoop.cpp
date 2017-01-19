#include "PIDLoop.h"

PIDLoop::PIDLoop() {
  k_p_Angle = .013;
  k_i_Angle = .001;
  k_d_Angle = .001;
  p_Angle = 0;
  i_Angle = 0;
  d_Angle = 0;
  angle_error = 0;
  angleOutput = 0;
  last_angle_error = 0;
  angleMaxError = 3;
  iteration_time = .005;
}

void PIDLoop::resetPID() {
	p_Angle = 0;
	i_Angle = 0;
	d_Angle = 0;
}

void PIDLoop::setAngle(float pAngleInput, float iAngleInput, float dAngleInput) {
	k_p_Angle = pAngleInput;
	k_i_Angle = iAngleInput;
	k_d_Angle = dAngleInput;
}

float PIDLoop::PIDAngle(float angleOffset, float desiredAngle) {
  //put in separate loop - not a while loop - keep checking and updating every runthrough of the normal loop - boolean for if this is running to stop you from manually moving the robot while the loop is running
  std::ofstream logger; logger.open("/var/loggerFile.txt", std::ofstream::out);
  logger << "Loop entered\n";

  angle_error = angleOffset - desiredAngle;
  angle_error = fabs(angle_error) > 180 ? -(angle_error - 180) : angle_error;

  p_Angle = k_p_Angle * angle_error;
  i_Angle += k_i_Angle * (angle_error * iteration_time);
  d_Angle = k_d_Angle * ((angle_error - last_angle_error) / iteration_time);
  angleOutput = p_Angle + i_Angle + d_Angle;
  last_angle_error = angle_error;

  //angleOutput = (angle_error / 180) * .7 + .2;


  angleOutput = fabs(angleOutput) < .23 ? std::copysign(.23, angleOutput) : angleOutput;
  angleOutput = fabs(angleOutput) > 1.0 ? std::copysign(1.0, angleOutput) : angleOutput;
  //angleOutput = angle_error < 0 ? angleOutput : -angleOutput;
  if (fabs(angle_error) < Constants::angleMaxError) {
	  i_Angle = 0;
	  angleOutput = 0;
  }
  angleOutput = -angleOutput;
  logger << p_Angle << " " << angle_error << " " << angleOutput << "\n";
  frc::Wait(iteration_time);
  logger.close();

  SmartDashboard::PutNumber("Accumulated i", i_Angle);
  SmartDashboard::PutNumber("Desired Angle", desiredAngle);
  SmartDashboard::PutNumber("angleOffset", angleOffset);
  SmartDashboard::PutNumber("angle_error", angle_error);

  return angleOutput;
}

float PIDLoop::PIDX(float distance, float angleOffset, float cameraOffset) {
  float k_p_X = .05;
  float k_i_X = .05;
  float k_d_X = .05;
  float p_X;
  float i_X = 0;
  float d_X;
  float x_error;
  float last_x_error = 0;
  float xOffset;
  float xOutput;
  float xMaxError = 3;

  std::ofstream logger; logger.open("/var/loggerFile.txt", std::ofstream::out);
  logger << "Loop entered\n";
  xOffset = distance * (sin(angleOffset) - (cameraOffset / 2)); //math is currently on my phone but will be put on google drive and in notebook


  x_error = xOffset;

  p_X = k_p_X * x_error;
  i_X += k_i_X * (x_error * iteration_time);
  d_X = k_d_X * ((x_error - last_x_error) / iteration_time);
  xOutput = p_X + i_X + d_X;
  last_x_error = x_error;

  xOffset = distance * (sin(angleOffset) - (cameraOffset / 2)); //math is currently on my phone but will be put on google drive and in notebook

  x_error = xOffset;

  xOutput = 0;

  frc::Wait(iteration_time);

  logger.close();

  return 0;
}
