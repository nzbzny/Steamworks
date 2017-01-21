#include "PIDLoop.h"

PIDLoop::PIDLoop() {
  k_p_Angle = .025;
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

  k_p_Y = .025;
  k_i_Y = .001;
  k_d_Y = .001;
  p_Y = 0;
  i_Y = 0;
  d_Y = 0;
  y_error = 0;
  last_y_error = 0;
  yOutput = 0;
  yMaxError = 6;

  k_p_X = .05;
  k_i_X = .05;
  k_d_X = .05;
  p_X = 0;
  i_X = 0;
  d_X = 0;
  x_error = 0;
  last_x_error = 0;
  xOutput = 0;
  xMaxError = 0;

}

void PIDLoop::resetPIDAngle() {
	p_Angle = 0;
	i_Angle = 0;
	d_Angle = 0;
}

void PIDLoop::setAngle(float pAngleInput, float iAngleInput, float dAngleInput) {
	k_p_Angle = pAngleInput;
	k_i_Angle = iAngleInput;
	k_d_Angle = dAngleInput;
}

void PIDLoop::setX(float pXInput, float iXInput, float dXInput) {
	k_p_X = pXInput;
	k_i_X = iXInput;
	k_d_X = dXInput;
}

void PIDLoop::setY(float pYInput, float iYInput, float dYInput) {
	k_p_Y = pYInput;
	k_i_Y = iYInput;
	k_d_Y = dYInput;
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

/*float PIDLoop::PIDX(float distance, float angleOffset, float cameraOffset) {
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
}*/

float PIDLoop::PIDX(float angleToGear) {
  std::ofstream logger; logger.open("/var/loggerFile.txt", std::ofstream::out);
  logger << "Loop entered\n";


  x_error = angleToGear;

  p_X = k_p_X * x_error;
  i_X += k_i_X * (x_error * iteration_time);
  d_X = k_d_X * ((x_error - last_x_error) / iteration_time);
  xOutput = p_X + i_X + d_X;
  last_x_error = x_error;

  xOutput = fabs(xOutput) > .7 ? std::copysign(.7, xOutput) : xOutput;
  xOutput = fabs(xOutput) < .2 ? std::copysign(.2, xOutput) : xOutput;

  if (x_error < xMaxError) {
	  xOutput = 0;
	  i_X = 0;
  }

  frc::Wait(iteration_time);

  logger.close();

  SmartDashboard::PutNumber("x_error", x_error);

  return xOutput;
}

float PIDLoop::PIDY(float lDistance, float rDistance) {
  float averageDistance;

  std::ofstream logger; logger.open("/var/loggerFile.txt", std::ofstream::out);
  logger << "Loop entered\n";
  if (lDistance > 0 && lDistance < 100 && rDistance > 0 && rDistance < 100) {
	  averageDistance = (lDistance + rDistance) / 2;
  }
  else if (lDistance < 0 || lDistance > 100) {
	  averageDistance = rDistance;
  } else if (rDistance < 0 || rDistance > 100) {
	  averageDistance = lDistance;
  } else {
	  SmartDashboard::PutString("PIDY Status", "Ultrasonic Error");
	  return 0;
  }
  y_error = averageDistance - 12;

  p_Y = k_p_Y * y_error;
  i_Y += k_i_Y * (y_error * iteration_time);
  d_Y = k_d_Y * ((y_error - last_y_error) / iteration_time);
  yOutput = p_Y + i_Y + d_Y;
  last_y_error = y_error;

  yOutput = fabs(yOutput) > .7 ? std::copysign(.7, yOutput) : yOutput;
  yOutput = fabs(yOutput) < .2 ? std::copysign(.2, yOutput) : yOutput;

  if (y_error < yMaxError) {
	  yOutput = 0;
	  i_Y = 0;
  }

  frc::Wait(iteration_time);

  logger.close();
  SmartDashboard::PutNumber("y_error", y_error);
  SmartDashboard::PutNumber("avgDist", averageDistance);
  SmartDashboard::PutNumber("i_Y", i_Y);


  return -yOutput;
}
