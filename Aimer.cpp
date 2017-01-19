#include "Aimer.h"

Aimer::Aimer()
{
	table = NetworkTable::GetTable("datatable");
}

float Aimer::GetAngleToGear()
{
	return (float)table->GetNumber("averageAzimuthOut-0", 42);
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

