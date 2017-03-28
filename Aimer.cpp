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
	return table->GetNumberArray("Azimuths-0", {});
}

std::vector<double> Aimer::GetRightAngleArray() {
	return table->GetNumberArray("Azimuths-1", {});
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
	float leftAngle = 0.0;
	float rightAngle = 0.0;
	float topAngle = 0.0;
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
/*	SmartDashboard::PutNumber("aimerLeftAngle", leftAngle);
	SmartDashboard::PutNumber("aimerRightAngle", rightAngle);
	SmartDashboard::PutNumber("topAngle", topAngle);
	SmartDashboard::PutNumber("leftDistance", leftDistance);
	SmartDashboard::PutNumber("perpendicular", perpendicular);
	SmartDashboard::PutNumber("dX", dX);
	SmartDashboard::PutNumber("dExt", dExt);
	SmartDashboard::PutNumber("dCTot", dCTot);
	SmartDashboard::PutNumber("center", center);
	SmartDashboard::PutNumber("centerAngle", centerAngle);*/
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

float Aimer::GetXDistanceToGear(float distance, float AngleError) {
	float angle = GetAngleToGear(distance);
	if (angle == 999)
		return 2000;
	float dExt;
//	float perpendicular;
//	float dCam = 15.75;
//	float xDistanceToGear;
	float adjustedAngle = angle - ConvertToPlusMinus180( AngleError);
	dExt = distance * tan(adjustedAngle * PI / 180);
	frc::SmartDashboard::PutNumber("Joe Current tgt angle", adjustedAngle);
	frc::SmartDashboard::PutNumber("joe dext", dExt);
//	SmartDashboard::PutNumber("aimerRightAngle", rightAngle);
//	SmartDashboard::PutNumber("tanLeft", tan(leftAngle * PI / 180));
//	SmartDashboard::PutNumber("tanRight", tan(rightAngle * PI / 180));
//	SmartDashboard::PutNumber("xDistanceToGear", xDistanceToGear);
//	SmartDashboard::PutNumber("perpendicular", perpendicular);
//	SmartDashboard::PutNumber("dExt", dExt);
	return dExt;
}

/*float Aimer::GetXDistanceToGear() {
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
}*/

//Audio Medic
float Aimer::GetAngleToGear(float distance)
{
	frc::SmartDashboard::PutNumber("Jpe Aimer getAngle Distance", distance);
	std::vector<double> l = table->GetNumberArray("Azimuths-0", {});
	std::vector<double> r = table->GetNumberArray("Azimuths-1", {});
	double lastLeft =  table->GetNumber("sinceLastUpdate-0", 999);
	frc::SmartDashboard::PutNumber("Joe last left age", lastLeft);
	double lastRight =  table->GetNumber("sinceLastUpdate-1", 999);
	frc::SmartDashboard::PutNumber("Joe last right age", lastRight);

	std::vector<double> angles;

	float angle = -999.0;

	for(uint x = 0; x < l.size(); x++) {
		for(uint y = 0; y < r.size(); y++) {

			if ((90 - l[x]) + (90 + (r[y])) < 185 && l[x] < 500 && r[y] < 500 && lastLeft < 1 && lastRight < 1) {
							//float intAngle = atan((tan(l[x]) +  tan(r[y]))/2);
							float intAngle = (l[x] + r[y])/2;
							angles.push_back(intAngle);
							//angles.push_back(atan(((1 / tan(l[x] * PI / 180)) + (1 / tan(r[x] * PI / 180))) / 2) * 180 / PI);
							frc::SmartDashboard::PutString("Joe AimerSelect", "two angles");
							return intAngle;
			}
			else if (l[x] < 100 && r[y] > 500 && lastLeft < 1 && distance != 0)
			{
				angle = atan((tan(l[x] * PI / 180) - Constants::leftCameraOffset / distance)) * 180 / PI;
				frc::SmartDashboard::PutNumber("Joe aimer left select", angle);
				frc::SmartDashboard::PutString("Joe AimerSelect", "left angles");
			}
			else if (l[x] > 100 && r[y] < 500 &&  lastRight < 1  && distance != 0)
			{
				angle = atan((tan(r[y] * PI / 180) - Constants::rightCameraOffset / distance) ) * 180 / PI;

				frc::SmartDashboard::PutString("Joe AimerSelect", "right angles");
			}else if (l[x] < 100 && lastLeft < 1  && distance != 0){
				angle = atan((tan(l[x] * PI / 180) - Constants::leftCameraOffset / distance)) * 180 / PI;

				frc::SmartDashboard::PutString("Joe AimerSelect", "left and did not make triangle");

			}



		}
	}
	frc::SmartDashboard::PutNumberArray("JOE ANGLE SELECT ARRAY", angles);
	if (angles.size() == 0)
		angle = 999;
	else if (angles.size() == 1) {
		angle = angles[0];
	} else {
		for (uint i = 0; i < angles.size(); i++)
			if(l.size() > 1){
				angle = (angles[i] > angle) ? angles[i] : angle;

			}else if(r.size() > 1){
				angle = (angles[i] < angle) ? angles[i] : angle;

			}
			else{

				angle = (fabs(angles[i]) < fabs(angle)) ? angles[i] : angle;
			}
	}
	if(angle = -999.0) angle = 999.0;
	return angle;
}

float Aimer::GetBoilerAngle()
{
	table = NetworkTable::GetTable("datatable-2");
	float angle = table->GetNumber("BoilerAzimuth-0", 0);
	table = NetworkTable::GetTable("datatable");
	return angle;
}

int Aimer::GetBoilerAge()
{
	table = NetworkTable::GetTable("datatable-2");
	float angle = table->GetNumber("BoilerAge-0", 2);
	table = NetworkTable::GetTable("datatable");
	return angle;
}

float Aimer::GetBoilerDistance()
{
	table = NetworkTable::GetTable("datatable-2");
	float angle = table->GetNumber("averageDistanceOut-0", 300);
	table = NetworkTable::GetTable("datatable");
	return angle;
}

float Aimer::ConvertToZeroTo360(float rawAngle){
	float result = rawAngle;
	result = (result >=360.0)? result - 360: result;
	result = (result <= -360)? result + 360: result;
	return result;
}
float Aimer::ConvertToPlusMinus180(float rawAngle){
	float result = rawAngle;
	result = (result >=360.0)? result - 360: result;
	result = (result <= -360)? result + 360: result;
	result = (result > 180)? result - 360: result;
	return result;
}
