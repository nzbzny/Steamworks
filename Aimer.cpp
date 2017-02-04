#include "Aimer.h"

Aimer::Aimer()
{
	table = NetworkTable::GetTable("datatable");
}

float Aimer::GetLeftAngleToGear()
{
	return (float)table->GetNumber("averageAzimuthOut-0", 42);
}

float Aimer::GetRightAngleToGear()
{
	return (float)table->GetNumber("averageAzimuthOut-1", 42);
}

std::vector<double> Aimer::GetLeftAngleArray() {
	table->GetNumberArray("Azimuths-0", {});
}

std::vector<double> Aimer::GetRightAngleArray() {
	table->GetNumberArray("Azimuths-1", {});
}

float Aimer::GetDistanceToGear()
{
	return (float)table->GetNumber("averageDistanceOut", 42);
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

float Aimer::TwoCameraAngleFilter() { //TODO: do math for if the gear is between the cameras
	float leftAngle;
	float rightAngle;
	float topAngle;
	float leftDistance;
	float perpendicular;
	float dX;
	float dExt;
	float dCTot;
	float center;
	float centerAngle;
	float dCam = 15.75; //TODO: I think this is a value but need to check
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

/*float Aimer::getXDistanceToGear() { //TODO: do math for if the gear is between the cameras
	float leftAngle = 90 - GetLeftAngleToGear();
	float rightAngle = 90 - GetRightAngleToGear();
	float topAngle;
	float leftDistance;
	float rightDistance;
	float perpendicular;
	float dX;
	float dExt;
	float dCam = 15.75; //TODO: I think this is a value but need to check
	if ((GetLeftAngleToGear() > 0 && GetRightAngleToGear() < 0) || (GetLeftAngleToGear() < 0 && GetRightAngleToGear() > 0)) {
		topAngle = 180 - (leftAngle + rightAngle);
		rightDistance = (dCam * sin(leftAngle * PI / 180)) / sin(topAngle * PI / 180);
		perpendicular = rightDistance * sin(rightAngle * PI / 180);
		dX = rightDistance * cos(rightAngle * PI / 180);
		SmartDashboard::PutNumber("aimerLeftAngle", leftAngle);
		SmartDashboard::PutNumber("aimerRightAngle", rightAngle);
		SmartDashboard::PutNumber("topAngle", topAngle);
		SmartDashboard::PutNumber("rightDistance", rightDistance);
		SmartDashboard::PutNumber("perpendicular", perpendicular);
		SmartDashboard::PutNumber("dX", dX);
		SmartDashboard::PutNumber("xDistanceToGear", (dCam / 2) - dX);
		return ((dCam / 2) - dX);
	} else {
		if (GetLeftAngleToGear() < 0 && GetRightAngleToGear() < 0) {
			leftAngle = 90 + GetLeftAngleToGear();
			rightAngle = 90 - GetRightAngleToGear();
			topAngle = leftAngle - rightAngle; //TODO: may have to flip signs
		} else if (GetLeftAngleToGear() > 0 && GetRightAngleToGear() > 0) {
			leftAngle = 90 - GetLeftAngleToGear();
			rightAngle = 90 + GetRightAngleToGear();
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
}*/

float Aimer::GetXDistanceToGear() {
	float leftAngle = 90 - fabs(GetLeftAngleToGear());
	float rightAngle = 90 - fabs(GetRightAngleToGear());
	float dExt;
	float perpendicular;
	float dCam = 15.75;
	float xDistanceToGear;
	perpendicular = ((1 / tan(leftAngle * PI / 180)) - dCam) / (1 / tan(rightAngle * PI / 180));
	dExt = perpendicular * (1 / tan(leftAngle * PI / 180)) - dCam;
	xDistanceToGear = dExt + dCam / 2;
	SmartDashboard::PutNumber("aimerLeftAngle", leftAngle);
	SmartDashboard::PutNumber("aimerRightAngle", rightAngle);
	SmartDashboard::PutNumber("tanLeft", tan(leftAngle * PI / 180));
	SmartDashboard::PutNumber("tanRight", tan(rightAngle * PI / 180));
	SmartDashboard::PutNumber("xDistanceToGear", xDistanceToGear);
	SmartDashboard::PutNumber("perpendicular", perpendicular);
	SmartDashboard::PutNumber("dExt", dExt);
	return 0;
}

//Audio Medic
float Aimer::GetAngleToGear()
{
	std::vector<double> l = table->GetNumberArray("Azimuths-0", {});
	std::vector<double> r = table->GetNumberArray("Azimuths-1", {});

	std::vector<double> angles;

	float angle = 1000.0, a = table->GetNumber("cameraBetweeness", 15.75);

	for(int x = 0; x < l.size(); x++) {
		for(int y = 0; y < r.size(); y++) {
			if ((90 - fabs(l[x])) + (90 + fabs(r[y])) < 185) {
				angles.push_back(atan(((1 / tan(l[x] * PI / 180)) + (1 / tan(r[x] * PI / 180))) / 2));
			}
		}
	}

	if (angles.size() == 1) {
		angle = angles[0];
	} else {
		for (int i = 0; i < angles.size(); i++)
		angle = (angles[i] < angle) ? angles[i] : angle;
	}
	return angle;
}
