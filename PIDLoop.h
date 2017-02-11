#include "WPILib.h"
#include "Constants.h"
#include "Filters.h"
#include <math.h>
#include <fstream>

#ifndef SRC_PIDLOOP_H
#define SRC_PIDLOOP_H

class PIDLoop {

  float k_p_Angle;
  float k_i_Angle;
  float k_d_Angle;
  float p_Angle;
  float i_Angle;
  float d_Angle;
  float angle_error;
  float last_angle_error;
  float angleOutput;
  float angleMaxError;
  float iteration_time;

  float k_p_Y = .05;
  float k_i_Y = .05;
  float k_d_Y = .05;
  float p_Y;
  float i_Y;
  float d_Y;
  float y_error;
  float last_y_error;
  float yOutput;
  float yMaxError;

  float k_p_X = .05;
  float k_i_X = .05;
  float k_d_X = .05;
  float p_X;
  float i_X = 0;
  float d_X;
  float x_error;
  float last_x_error = 0;
  float xOutput;
  float xMaxError = 0;

  float lastLeftUltrasonic = 0;
  float lastRightUltrasonic = 0;

  Filters filter;

public:
	PIDLoop();
	void resetPIDAngle();
	void resetPIDX();
	void resetPIDY();
	void setAngle(float pAngleInput, float iAngleInput, float dAngleInput);
	void setX(float pXInput, float iXInput, float dXInput);
	void setY(float pYInput, float iYInput, float dYInput);
	float PIDAngle(float yaw, float desiredAngle);
	//float PIDX(float distance, float angleOffset, float cameraOffset);
	float PIDX(float angleToGear);
	float PIDY(float lDistance, float rDistance);
	float ultrasonicFilter(float left, float right);
};

#endif
