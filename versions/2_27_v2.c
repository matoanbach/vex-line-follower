#pragma config(Sensor, in1,    rightSensor,    sensorLineFollower)
#pragma config(Sensor, in2,    centerSensor,   sensorLineFollower)
#pragma config(Sensor, in8,    leftSensor,     sensorLineFollower)
#pragma config(Sensor, dgtl5,  leftBumper,     sensorTouch)
#pragma config(Sensor, dgtl6,  rightBumper,    sensorTouch)
#pragma config(Sensor, dgtl8,  sonarSensor,    sensorSONAR_cm)
#pragma config(Motor,  port1,           leftMotor,     tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port10,          rightMotor,    tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

bool motorDisabled = true;
bool bClickInProgress = false;

void processButton()
{
	if (bClickInProgress)
	{
		if (SensorValue[leftBumper])
			bClickInProgress = false;
	}
	else
	{
		if (!SensorValue[leftBumper])
		{
			motorDisabled = !motorDisabled;
			bClickInProgress = true;
		}
	}
}

task main() {

	while(true) {
		processButton();
		if (motorDisabled)
		{
			motor[leftMotor]  = 0;
			motor[rightMotor] = 0;
		}
		else
		{
			motor[leftMotor]  = -150;
			motor[rightMotor] = 50;
		}
	}
}
