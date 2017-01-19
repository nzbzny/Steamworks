#include "WPILib.h"
#include "Constants.h"
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

public:
	PIDLoop();
	void resetPID();
	void setAngle(float pAngleInput, float iAngleInput, float dAngleInput);
	float PIDAngle(float yaw, float desiredAngle);
	float PIDX(float distance, float angleOffset, float cameraOffset);
};

#endif /* SRC_ROBOT_H_ */
