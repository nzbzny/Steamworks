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

float Aimer::twoCameraAngleFilter() {
	return -(GetLeftAngleToGear() + GetRightAngleToGear()) / 2.0;
}
