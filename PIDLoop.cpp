#include "PIDLoop.h"

PIDLoop::PIDLoop() //:
	//filter()
{
  k_p_Angle = .01;
  k_i_Angle = .001;
  k_d_Angle = .001;
  p_Angle = 0;
  i_Angle = 0;
  d_Angle = 0;
  angle_error = 0;
  angleOutput = 0;
  last_angle_error = 0;
  angleMaxError = 2;
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
  xMaxError = 3; //TODO: may have to play around with this number
}

void PIDLoop::resetPIDAngle() { //reset angle pid values
	p_Angle = 0;
	i_Angle = 0;
	d_Angle = 0;
}

void PIDLoop::resetPIDX() { //reset x pid values
  p_X = 0;
  i_X = 0;
  d_X = 0;
}

void PIDLoop::resetPIDY() { //reset y pid values
  p_Y = 0;
  i_Y = 0;
  d_Y = 0;
}

void PIDLoop::setAngle(float pAngleInput, float iAngleInput, float dAngleInput) { //set angle PID constants
	k_p_Angle = pAngleInput;
	k_i_Angle = iAngleInput;
	k_d_Angle = dAngleInput;
}

void PIDLoop::setX(float pXInput, float iXInput, float dXInput) { //set x PID constants
	k_p_X = pXInput;
	k_i_X = iXInput;
	k_d_X = dXInput;
}

void PIDLoop::setY(float pYInput, float iYInput, float dYInput) { //set y PID constants
	k_p_Y = pYInput;
	k_i_Y = iYInput;
	k_d_Y = dYInput;
}
float PIDLoop::PIDAngle(float angleOffset, float desiredAngle) {
  //put in separate loop - not a while loop - keep checking and updating every runthrough of the normal loop - boolean for if this is running to stop you from manually moving the robot while the loop is running
  std::ofstream logger; logger.open("/var/loggerFile.txt", std::ofstream::out); //start logger
  logger << "Loop entered\n";

  angle_error = angleOffset - desiredAngle; //calculate error
  angle_error = fabs(angle_error) > 180 ? 180 - angle_error : angle_error; //scale error to take shortest path
  if (desiredAngle == 0 && angleOffset > 180) {
		  angle_error = angleOffset - 360;
  }

  p_Angle = k_p_Angle * angle_error; //calculate p
  i_Angle += k_i_Angle * (angle_error * iteration_time); //calculate i
  d_Angle = k_d_Angle * ((angle_error - last_angle_error) / iteration_time); //calculate d
  angleOutput = p_Angle + i_Angle + d_Angle; //calculate output
  last_angle_error = angle_error; //set last angle error for d value



  angleOutput = fabs(angleOutput) < .1 ? std::copysign(.1, angleOutput) : angleOutput; //if angleOutput is below min, set to min
  angleOutput = fabs(angleOutput) > .9 ? std::copysign(.9, angleOutput) : angleOutput; //if angleOutput is above max, set to max
  //angleOutput = angle_error < 0 ? angleOutput : -angleOutput;
  if (fabs(angle_error) < Constants::angleErrorLimit) { //if done moving
	  i_Angle = 0;
	  angleOutput = 0;
  }
  angleOutput = -angleOutput; //TODO: may need to add back in
  logger << p_Angle << " " << angle_error << " " << angleOutput << "\n"; //output to log file
  //frc::Wait(iteration_time);
  logger.close();

  /*SmartDashboard::PutNumber("Accumulated i", i_Angle);
  SmartDashboard::PutNumber("Desired Angle", desiredAngle);
  SmartDashboard::PutNumber("angleOffset", angleOffset);
  SmartDashboard::PutNumber("angle_error", angle_error);*/

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
  std::ofstream logger; logger.open("/var/loggerFile.txt", std::ofstream::out); //open logger
  logger << "Loop entered\n";


  x_error = angleToGear; //error value

  p_X = k_p_X * x_error; //calculate p
  i_X += k_i_X * (x_error * iteration_time); //calculate i
  d_X = k_d_X * ((x_error - last_x_error) / iteration_time); //calculate d
  xOutput = p_X + i_X + d_X; //calculate output
  last_x_error = x_error; //set last x error for d value

  xOutput = fabs(xOutput) > .7 ? std::copysign(.7, xOutput) : xOutput; //if xOutput is above max, set to max
  xOutput = fabs(xOutput) < .2 ? std::copysign(.2, xOutput) : xOutput; //if xOutput is below min, set to min

  if (fabs(x_error) < xMaxError) { //if done
	  xOutput = 0;
	  i_X = 0;
  }

  //frc::Wait(iteration_time);
  //frc::Wait(.005);

  logger.close(); //close logger

  //SmartDashboard::PutNumber("x_error", x_error);

  return xOutput;
}

float PIDLoop::PIDY(float lDistance, float rDistance) {
  float averageDistance;

  std::ofstream logger; logger.open("/var/loggerFile.txt", std::ofstream::out); //start logger
  logger << "Loop entered\n";
  //TODO: put these checks into the ultrasonic filter function
  //averageDistance = filter.ultrasonicFilter(lDistance, rDistance);
  averageDistance = 12; //TODO: why is this 12? @Joe
  if (averageDistance == -1) {
	  return -1;
  }
  y_error = averageDistance - 12; //error - stop about a foot away

  p_Y = k_p_Y * y_error; //calculate p
  i_Y += k_i_Y * (y_error * iteration_time); //calculate i
  d_Y = k_d_Y * ((y_error - last_y_error) / iteration_time); //calculate d
  yOutput = p_Y + i_Y + d_Y; //calculate yOutput
  last_y_error = y_error; //set last y error for d value

  yOutput = fabs(yOutput) > .7 ? std::copysign(.7, yOutput) : yOutput; //if above max set to max
  yOutput = fabs(yOutput) < .2 ? std::copysign(.2, yOutput) : yOutput; //if below min set to min

  if (y_error < yMaxError) { //if able to stop
	  yOutput = 0;
	  i_Y = 0;
  }

  //frc::Wait(iteration_time);

  logger.close(); //close logger
  /*SmartDashboard::PutNumber("y_error", y_error);
  SmartDashboard::PutNumber("avgDist", averageDistance);
  SmartDashboard::PutNumber("i_Y", i_Y);*/

  return -yOutput;
}

float PIDLoop::ultrasonicFilter(float left, float right) { //to smooth out the ultrasonic values
	float leftOutput = 0;
	float rightOutput = 0;
	float gain = .2;

	leftOutput = gain * left + (1 - gain) * lastLeftUltrasonic; //larry math
	rightOutput = gain * right + (1 - gain) * lastRightUltrasonic;

	lastLeftUltrasonic = left;
	lastRightUltrasonic = right;

	return (leftOutput + rightOutput) / 2; //average
}
