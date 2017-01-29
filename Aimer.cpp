#include "Aimer.h"

Aimer::Aimer()
{
	table = NetworkTable::GetTable("datatable");
	table->SetUpdateRate(.02); //20ms refresh rate
}

float Aimer::GetLeftAngleToGear()
{
	return (float)table->GetNumber("averageAzimuthOut-0", 42);
}

float Aimer::GetRightAngleToGear()
{
	return (float)table->GetNumber("averageAzimuthOut-1", 42);
}


float Aimer::GetOffset() {
	return 0;
}

float Aimer::GetSpeedToShoot()
{
	return 42;
}

float Aimer::GetAngleToShoot()
{
	return (float)table->GetNumber("averageShootyAngleOut", 42);
}

int Aimer::GetAge()
{
	return (int)table->GetNumber("sinceLastUpdate", 4);
}

float Aimer::Distancinator()
{
	float lAngle = 90.0 - fabs(table->GetNumber("averageAzimuthOut-0", 0.0));
	float rAngle = 90.0 - fabs(table->GetNumber("averageAzimuthOut-1", 0.0));
	float distanceBetweenCameras = SmartDashboard::GetNumber("cameraBetweeness", 15.75);
	return (float)(sin(3.1415926*lAngle/180) * sin(3.1415926*rAngle/180) * distanceBetweenCameras) / sin(3.1415926*(180 - (lAngle + rAngle))/180);
}

void Aimer::DeleteUnused() {
}

float Aimer::twoCameraAngleFilter() { //TODO: do math for if the gear is between the cameras
	float leftAngle = fabs(GetLeftAngleToGear());
	float rightAngle = fabs(GetRightAngleToGear());
	float topAngle;
	float leftDistance;
	float perpendicular;
	float dX;
	float dExt;
	float dCTot;
	float center;
	float centerAngle;
	float dCam = 18.5; //TODO: I think this is a value but need to check
	if (leftAngle < 0 && rightAngle < 0) {
		topAngle = leftAngle - rightAngle; //TODO: may have to flip signs
	} else if (leftAngle > 0 && rightAngle > 0) {
		topAngle = rightAngle - leftAngle; //TODO: may have to flip signs
	}
	leftDistance = (dCam * sin(rightAngle * PI / 180)) / sin(topAngle * PI / 180);
	perpendicular = leftDistance * sin(leftAngle * PI / 180);
	dX = sqrt(pow(leftDistance, 2) - pow(perpendicular, 2));
	dExt = dX - dCam;
	dCTot = (dCam / 2) + dExt;
	center = sqrt(pow(dCTot, 2) - pow(perpendicular, 2));
	centerAngle = asin(perpendicular / center) * 180 / PI;
	SmartDashboard::PutNumber("aimerLeftAngle", leftAngle);
	SmartDashboard::PutNumber("aimerRightAngle", rightAngle);
	SmartDashboard::PutNumber("topAngle", topAngle);
	SmartDashboard::PutNumber("leftDistance", leftDistance);
	SmartDashboard::PutNumber("perpendicular", perpendicular);
	SmartDashboard::PutNumber("dX", dX);
	SmartDashboard::PutNumber("dExt", dExt);
	SmartDashboard::PutNumber("dCTot", dCTot);
	SmartDashboard::PutNumber("center", center);
	SmartDashboard::PutNumber("centerAngle", centerAngle);
	return centerAngle;
}

float Aimer::getXDistanceToGear() { //TODO: do math for if the gear is between the cameras
	float leftAngle = fabs(GetLeftAngleToGear());
	float rightAngle = fabs(GetRightAngleToGear());
	float topAngle;
	float leftDistance;
	float perpendicular;
	float dX;
	float dExt;
	float dCam = 18.5; //TODO: I think this is a value but need to check
	if (leftAngle < 0 && rightAngle < 0) {
		topAngle = leftAngle - rightAngle; //TODO: may have to flip signs
	} else if (leftAngle > 0 && rightAngle > 0) {
		topAngle = rightAngle - leftAngle; //TODO: may have to flip signs
	}
	leftDistance = (dCam * sin(rightAngle * PI / 180)) / sin(topAngle * PI / 180);
	perpendicular = leftDistance * sin(leftAngle * PI / 180);
	dX = sqrt(pow(leftDistance, 2) - pow(perpendicular, 2));
	dExt = dX - dCam;
	SmartDashboard::PutNumber("aimerLeftAngle", leftAngle);
	SmartDashboard::PutNumber("aimerRightAngle", rightAngle);
	SmartDashboard::PutNumber("topAngle", topAngle);
	SmartDashboard::PutNumber("leftDistance", leftDistance);
	SmartDashboard::PutNumber("perpendicular", perpendicular);
	SmartDashboard::PutNumber("dX", dX);
	SmartDashboard::PutNumber("dExt", dExt);
	SmartDashboard::PutNumber("xDistanceToGear", dExt + (dCam / 2));
	return dExt + (dCam / 2); //distance to the middle of the robot
}
