#pragma config(Sensor, in1,    lineFollowerRIGHT, sensorLineFollower)
#pragma config(Sensor, in2,    lineFollowerCENTER, sensorLineFollower)
#pragma config(Sensor, in8,    lineFollowerLEFT, sensorReflection)
#pragma config(Motor,  port1,           leftMotor,     tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port10,          rightMotor,    tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*----------------------------------------------------------------------------------------------------*\
|*                             	    - Triple Sensor Line Tracking -                                   *|
|*                                      ROBOTC on VEX 2.0 CORTEX                                      *|
|*                                                                                                    *|
|*  This program uses 3 VEX Line Follower Sensors to track a black line on a light(er) surface.       *|
|*  There is a two second pause at the beginning of the program.                                      *|
|*                                                                                                    *|
|*                                        ROBOT CONFIGURATION                                         *|
|*    NOTES:                                                                                          *|
|*    1)  Reversing 'rightMotor' (port 2) in the "Motors and Sensors Setup" is needed with the        *|
|*        "Squarebot" mode, but may not be needed for all robot configurations.                       *|
|*    2)  Lighting conditions, line darkness, and surface lightness change from place to place,       *|
|*        so the value of 'threshold' may need to be changed to better suit your environment.         *|
|*                                                                                                    *|
|*    MOTORS & SENSORS:                                                                               *|
|*    [I/O Port]          [Name]              [Type]                [Description]                     *|
|*    Motor  - Port 2     rightMotor          VEX 3-wire module     Right side motor                  *|
|*    Motor  - Port 3     leftMotor           VEX 3-wire module     Left side motor                   *|
|*    Analog - Port 1     lineFollowerRIGHT   VEX Light Sensor      Front-right, facing down          *|
|*    Analog - Port 2     lineFollowerCENTER  VEX Light Sensor      Front-center, facing down         *|
|*    Analog - Port 3     lineFollowerLEFT    VEX Light Sensor      Front-left, facing down           *|
\*-----------------------------------------------------------------------------------------------4246-*/


//+++++++++++++++++++++++++++++++++++++++++++++| MAIN |+++++++++++++++++++++++++++++++++++++++++++++++
task main()
{
	wait1Msec(2000);          // The program waits for 2000 milliseconds before continuing.

	int black_threshold = 2250;      /* found by taking a reading on both DARK and LIGHT    */
	/* surfaces, adding them together, then dividing by 2. */
	int red_threshold = 1475;
	while(true)
	{

		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -+
		//displayLCDCenteredString(0, "LEFT  CNTR  RGHT");        //  Display   |
		//displayLCDPos(1,0);                                     //  Sensor    |
		//displayNextLCDNumber(SensorValue(lineFollowerLEFT));    //  Readings  |
		//displayLCDPos(1,6);                                     //  to LCD.   |
		//displayNextLCDNumber(SensorValue(lineFollowerCENTER));  //            |
		//displayLCDPos(1,12);                                    //  L  C  R   |
		//displayNextLCDNumber(SensorValue(lineFollowerRIGHT));   //  x  x  x   |
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -+

		// RIGHT sensor sees dark:
		//turn right
		//red threshold -> 1475
		// red: 1550
		// white: 1400

		//turn left
		//motor[leftMotor] = 35;
		//motor[rightMotor] = 55;

		if
			(
		(SensorValue(lineFollowerRIGHT) > black_threshold) &&
		(SensorValue(lineFollowerCENTER) > black_threshold) &&
		(SensorValue(lineFollowerLEFT) < 1450)
		)
		{
			// counter-steer right:
			motor[leftMotor]  = 35;
			motor[rightMotor] = -35;
		}
		// CENTER sensor sees dark:
		else if
			(
		(SensorValue(lineFollowerRIGHT) < 1450) &&
		(SensorValue(lineFollowerCENTER) > black_threshold) &&
		(SensorValue(lineFollowerLEFT) < 1450))
		{
			// go straight
			motor[leftMotor]  = 50;
			motor[rightMotor] = 50;
		}
		// LEFT sensor sees dark:
		else if
			(
		(SensorValue(lineFollowerRIGHT) < 1450) &&
		(SensorValue(lineFollowerCENTER) > black_threshold) &&
		(SensorValue(lineFollowerLEFT) > black_threshold)
		)
		{
			// counter-steer left:
			motor[leftMotor]  = -45;
			motor[rightMotor] = 45;
		}
		else if
			(
		(SensorValue(lineFollowerRIGHT) > red_threshold && SensorValue(lineFollowerRIGHT) < 1600) &&
		(SensorValue(lineFollowerCENTER) > red_threshold && SensorValue(lineFollowerCENTER) < 1600) &&
		(SensorValue(lineFollowerLEFT) < red_threshold)
		) {
			motor[leftMotor] = 55;
			motor[rightMotor] = -55;

			wait1Msec(1500);

			motor[leftMotor] = 55;
			motor[rightMotor] = 55;

			wait1Msec(2000);
		}
	}
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
