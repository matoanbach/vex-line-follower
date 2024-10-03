#pragma config(Sensor, in1,    lineFollowerRIGHT, sensorLineFollower)
#pragma config(Sensor, in2,    lineFollowerCENTER, sensorLineFollower)
#pragma config(Sensor, in8,    lineFollowerLEFT, sensorReflection)
#pragma config(Sensor, dgtl5,  leftBumper,     sensorTouch)
#pragma config(Motor,  port1,           leftMotor,     tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port10,          rightMotor,    tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//================================================
//=========This part is for on/off button only ===
//================================================
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
//================================================
//============End of on/off button================
//================================================




//================================================
//======== Common variables for PID ==============
//================================================
int error;
int lastError = 0;
int integral = 0;
int derivative = 0;


int turn;
int nLeftMotor;
int nRightMotor;

int offset = 1351;
int Tp = 60; //40
int Kp =  30; //10
int Ki =   7;
int Kd = 45; //45

int lightValue;

//================================================
//======== End of variables for PID ==============
//================================================


//================================================
//======== get error for Proportional ============
//================================================


//================================================
//======== End of getting error for Proportional==
//================================================


int getLightValue()
{
	int light1 = SensorValue(lineFollowerLEFT);
	int light2 = SensorValue(lineFollowerCENTER);
	int light3 = SensorValue(lineFollowerRIGHT);
	int total = light1 + light2 + light3;
	return total / 3;
}

//================================================
//======== End of get error for Proportional =====
//================================================

task main()
{
	wait1Msec(2000);
	while(true) {
		processButton();
		lightValue = getLightValue();
		error = lightValue - offset;
		integral = (2/3)*integral + error;
		derivative = error - lastError;

		turn = (Kp*error + Ki*integral + Kd*derivative)/1000;
		/*
		if (turn > 0)//turn right if turn > 0
		{
		nLeftMotor  =  - Tp - turn;
		nRightMotor = Tp - (turn/8); //8
		}
		else				//turn left if turn < 0
		{
		nLeftMotor  = - Tp - (turn/8);
		nRightMotor = Tp - turn;
		}
		*/

		if (turn > 0)
		{
			nLeftMotor  = Tp + turn;
			nRightMotor = Tp - 10 * turn / 2; //8
		}
		else
		{
			nLeftMotor  = Tp + 10 * turn / 2;
			nRightMotor = Tp - turn;
		}

		if (motorDisabled)
		{
			motor[leftMotor]  = 0;
			motor[rightMotor] = 0;
		}
		else
		{
			motor[leftMotor]  = nLeftMotor;
			motor[rightMotor] = nRightMotor;
		}
		lastError = error;
	}
}
