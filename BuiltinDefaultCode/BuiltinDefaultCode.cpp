#if 0
Driver stick 1 interchangible with driver stick 2
2 : Shift Down
3 : Shift Up
4: Auto Aim same as 5
5 : Auto Aim same as 4
8 : Arcade
9 : Tank

Driver stick 1 interchangible with driver stick 2
2 : Shift Down
3 : Shift Up
4 : Auto Aim same as 5
5 : Auto Aim same as 4
8 : Arcade
9 : Tank

Operator Stick :
1 : Fire
2 : Top Goal
4 : Left Middle Goal
5 : Right Middle Goal
6 : Shooter Up
7 : Shooter Down
10 : Dump same as 11
11 :Dump same as 10
z : Blast/manual mode

Steering Wheel
Any Button : enable the wheel
#endif
#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"
#include "Vision/Axiscamera.h"
#include <DriverStation.h>

//Drive program for 2013 team 668 kit bot


#define BOTDRIVE
#define BOTDRIVEBUTTON
#define PRINT
#define TIMERS
#define ENCODERS
#define VISIONSYSTEM
#define SHIFTERS
#define DS
//#define OPTICAL_SENSOR_TEST
//#define CAMERA
#define HOMEBREWPID
//#define WPILIBPID

#if defined(HOMEBREWPID) && defined(WPILIBPID)
#error Please disable either HOMEBREWPID or WPILIBPID
#endif

#define STATEMACHINE

// Working values for HOMEBREWPID:
//P: 0.070
//I: 0.00
//D: ~0.500
//SampleRate: 0.729
//SampleRate (s): .729 / 5 * (1.0 - 0.001) + 0.001 = .1467 seconds


// Camera Dimensions
// 17.5" forward from hinge of shooter to lens
// 8.5"  left of center of axle for shooter (center of hinge)
// 15.25" vertical to center of lens to ground


//Camera constants used for distance calculation
#define X_IMAGE_RES 320		//X Image resolution in pixels, should be 160, 320 or 640
//#define VIEW_ANGLE 48		//Axis 206 camera
#define VIEW_ANGLE 43.5  //Axis M1011 camera
//Using M10011 Camera FRC 2013
#define PI 3.1415926535897932384626433832795028841971

//Score limits used for target identification
#define RECTANGULARITY_LIMIT 60
#define ASPECT_RATIO_LIMIT 75
#define X_EDGE_LIMIT 40
#define Y_EDGE_LIMIT 60

//Minimum area of particles to be considered
#define AREA_MINIMUM 500

//Edge profile constants used for hollowness score calculation
#define XMAXSIZE 24
#define XMINSIZE 24
#define YMAXSIZE 24
#define YMINSIZE 48
//Todo
//insert desciption of arrays below
const double xMax[XMAXSIZE] =
{ 1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5,
		1, 1, 1, 1 };
const double xMin[XMINSIZE] =
{ .4, .6, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1,
		.1, .1, .1, 0.6, 0 };
const double yMax[YMAXSIZE] =
{ 1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5,
		1, 1, 1, 1 };
const double yMin[YMINSIZE] =
{ .4, .6, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
		.05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
		.05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
		.05, .05, .6, 0 };

// used to control how often (how many loops) prints happen:
#define PRINTRATE (30)

// These are crude State machine wait times
#define SHOOTER_CYLINDER_RETRACT_TIME 0.2 // seconds, a guess
#define SHOOTER_CYLINDER_EXTEND_TIME  0.2 // seconds, a guess
#define SHOOTER_MOTOR_SPINUP_WAIT     4.0 // seconds, a guess
#define DUMP_RPM 3000.0

#define PRINT_BUFFER_SIZE 50

// This is here so that if we need to change the motor controllers easily, we can
#if 1
#define MOTOR_CONTROLLER Talon
#else
#define MOTOR_CONTROLLER Victor
#endif

//assuming that both motors are going the same speed in the same direction
#define GET_MOTOR_SPEED (-ShooterMotorLeft->Get())

// We are doing this so that if the Optical sensor outputs are inverted, then
// we don't have to switch all of the occurrences of these calls
#define FIRST_OPTICAL_SENSOR_VALUE  (!FirstOpticalSensor->Get())
#define SECOND_OPTICAL_SENSOR_VALUE (!SecondOpticalSensor->Get())

// here are the solenoid port defines
#define SHIFT_UP_STATE 1
#define SHIFT_OUT		5
#define SHIFT_IN		6

#define LIFTER_SHIFT_IN		3
#define LIFTER_SHIFT_OUT	4

#define LOADER_SHIFT_IN		2
#define LOADER_SHIFT_OUT	1

// Here is the definition of all of the PWM's

// F=front, B=back, L=left, R=right
#define DRIVE_MOTOR_F_L		2
#define DRIVE_MOTOR_B_L		1
#define DRIVE_MOTOR_F_R		6
#define DRIVE_MOTOR_B_R		5

#define SHOOTER_MOTOR_LEFT	3
#define SHOOTER_MOTOR_RIGHT	4

#define MOTOR_MAX -1.0f // motor gearing reverses motor
#define MOTOR_STOP 0.0f

// here is the definition of all of the Digital IO's
#define FIRST_OPTICAL_SENSOR	13
#define SECOND_OPTICAL_SENSOR	14
#define PRESSURE_SWITCH     4



//here is where the relays are defined
#define COMPRESSOR_RELAY    1



// Joystick 1 is the center joystick (arcade and tank).
// Joystick 2 is the left joystick (tank).
// Joystick 3 is the right joystick (operator).
// Joystick 4 is a provision for the steering wheel
#define JOYSTICK1 1
#define JOYSTICK2 2
#define JOYSTICK3 3
#define JOYSTICK4 4

/*
 * Wheel Buttons:
 * A = 1
 * B = 2
 * C = 3
 * X = 4
 * Y = 5
 * Z = 6
 * 
 * Turn = X axis
 * left = -1.0
 * right = 1.0
 */

// here is where the defines go for buttons
//Drive Stick Left
#define DISC_TIMER_RESET	1  // for optical sensors measuring Frisbee speed, not operational 'bot

#define SHIFT_UP_BUTTON		2  //also on Drive Stick Right
#define SHIFT_DOWN_BUTTON	3  //also on Drive Stick Right

#define ARCADE_BUTTON		8  //also on Drive Stick Right
#define TANK_BUTTON			9  //also on Drive Stick Right



//Drive Stick Right: see Drive Stick Left!

// Shooter Stick

#define PRINT_ENCODER        4
#define SHOOTER_UP           6
#define SHOOTER_DOWN         7
#define START_STATE_MACHINE  8
#define STATE_MACHINE_MANUAL 9
#define STATE_MACHINE_DUMPa 10
#define STATE_MACHINE_DUMPb 11

//TODO: also using ShooterStick Z for shooting!
#if defined(HOMEBREWPID) || defined (WPILIBPID)
//#define kP (1 - DriveStickL->GetZ())
//#define kI (1 - DriveStickR->GetZ())
//#define kD (1 - ShooterStick->GetZ())
//#define kF 1
#define kPID_RANGE .001 + (5.0 - .001)
#endif

#define INpS_TO_FpS(a) ((a)/12.0)
#define INpS_TO_FUpFN(a) ((a)*(3600.0*60.0*24.0*14.0)/(220.0*12.0))

#define FORTNIGHT
#ifdef FORTNIGHT
#define CONVERT(a) (INpS_TO_FUpFN(a))
#else
#define CONVERT(a) (INpS_TO_FpS(a))
#endif

#define FRISBEE_DIA 11.0
#define OPTICAL_SENSOR_DISTANCE 2.5

#define MAX_RPM 5000.0

class RobotDemo : public SimpleRobot
{
#ifdef VISIONSYSTEM
	//Structure to represent the scores for the various tests used for target identification
	struct Scores
	{
		double rectangularity;
		double aspectRatioInner;
		double aspectRatioOuter;
		double xEdge;
		double yEdge;
	};
	static const char NON_GOAL = 0;
	static const char MID_GOAL = 1;
	static const char TOP_GOAL = 2;
	struct FinalReports
	{
		ParticleAnalysisReport report;
		// High middle or low goal
		char type;
	};
	struct FinalReports *finalreports[16];
#endif
	//declare classes and stuff here. 
	Joystick *DriveStickL, *DriveStickR, *ShooterStick, *Wheel;
	MOTOR_CONTROLLER *ShooterMotorLeft;
	MOTOR_CONTROLLER *ShooterMotorRight;

#ifdef BOTDRIVE
	RobotDrive *drive;
	MOTOR_CONTROLLER *frontRight;
	MOTOR_CONTROLLER *frontLeft;
	MOTOR_CONTROLLER *backRight;
	MOTOR_CONTROLLER *backLeft;
#endif
	Scores *score;
	DigitalInput *FirstOpticalSensor;
	DigitalInput *SecondOpticalSensor;
#ifdef TIMERS
	Timer *DiscTimer;
	Timer *VisionTimer;
	Timer *OpticalSensorTimer;
	Timer *RPMTimer;
#endif
#ifdef ENCODERS
	Encoder *encoderShooterMotor;
#endif
#ifdef SHIFTERS
	DoubleSolenoid *shift;
	DoubleSolenoid *shooterAngleController;
	DoubleSolenoid *loaderSolenoid;
	Compressor *air;
#endif
#ifdef DS
	DriverStation *ds;
	DriverStationLCD *dslcd;
#endif
#ifdef CAMERA
	AxisCamera *camera;
#endif
#ifdef HOMEBREWPID
	Timer *PIDTimer;
	Timer *PIDSampleTimer;
	float error, accumerror;
	float P, I, D, F;
	float p, oldp;
	unsigned int printCount;
#endif
	int prevEnc;
	double prevTime;
#ifdef WPILIBPID
	PIDController *pid;
#endif

	char printbuffer [2][PRINT_BUFFER_SIZE+1];

	double kP, kI, kD, kF, sampleRate;
	int rpm, target;
	enum shooterState
	{	start, idle, startShooterMotor, shooterMotorSpinupWait, extendCylinder, extendCylinderWait, retractCylinder, retractCylinderWait, checkFrisbeeCount, stopShooterMotor, shooterCheckTrigger, setAutoShoot, setManualShoot, setDumpShoot};
	enum shooterMode
	{	dump, manualShoot, autoShoot, autonomous, noMode};
	Timer *shooterTimer;

public:
	RobotDemo()
	{
		DriveStickL=new Joystick(JOYSTICK1);
		DriveStickR=new Joystick(JOYSTICK2);
		ShooterStick=new Joystick(JOYSTICK3);
		Wheel = new Joystick(JOYSTICK4);

		ShooterMotorLeft=new MOTOR_CONTROLLER(SHOOTER_MOTOR_LEFT);
		ShooterMotorRight= new MOTOR_CONTROLLER(SHOOTER_MOTOR_RIGHT);

#ifdef OPTICAL_SENSOR_TEST
		/**************** OPTICAL SENSORS READ TRUE if no frisbee
		 * and FALSE if frisbee (black wire connected to digital input)
		 */

		FirstOpticalSensor = new DigitalInput(FIRST_OPTICAL_SENSOR);
		SecondOpticalSensor = new DigitalInput(SECOND_OPTICAL_SENSOR);
#endif
#ifdef ENCODERS
		encoderShooterMotor = new Encoder(11,12,true);
#endif
#ifdef CAMERA
		camera = &(AxisCamera::GetInstance("10.6.68.11"));
		//printf("Camera* is %d bytes, = 0x%x\n",sizeof(camera),int(camera));
		camera->WriteResolution(AxisCamera::kResolution_640x480);
#endif
#ifdef SHIFTERS
		shift = new DoubleSolenoid(SHIFT_IN, SHIFT_OUT);
		shooterAngleController = new DoubleSolenoid(LIFTER_SHIFT_IN, LIFTER_SHIFT_OUT);
		loaderSolenoid = new DoubleSolenoid(LOADER_SHIFT_IN, LOADER_SHIFT_OUT);
		air= new Compressor (PRESSURE_SWITCH, COMPRESSOR_RELAY);
		air->Start();

#endif
#ifdef BOTDRIVE
		frontLeft = new MOTOR_CONTROLLER(DRIVE_MOTOR_F_L);
		backLeft = new MOTOR_CONTROLLER(DRIVE_MOTOR_B_L);
		frontRight = new MOTOR_CONTROLLER(DRIVE_MOTOR_F_R);
		backRight = new MOTOR_CONTROLLER(DRIVE_MOTOR_B_R);
		drive = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
#if 0   // provision for reversing motors:
		drive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);

		/* note: two motors in RobotDrive appear to be 
		 * rear motors, not the front ones, per the lines below:
		 */
		drive->SetInvertedMotor(RobotDrive::kRearRightMotor, true); // controls our bot
		drive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true); // controls our bot

#endif
		drive->SetSafetyEnabled(false);
#endif
#ifdef TIMERS
		DiscTimer = new Timer();
		OpticalSensorTimer = new Timer();
		RPMTimer = new Timer();
#endif
#ifdef VISIONSYSTEM
		VisionTimer = new Timer();
#endif
#ifdef DS
		ds = DriverStation::GetInstance();
		dslcd = DriverStationLCD::GetInstance();
#endif
#ifdef HOMEBREWPID
		PIDTimer = new Timer();
		PIDSampleTimer = new Timer();
		accumerror = 0;
		error = 0;
#endif
#ifdef WPILIBPID
		pid = new PIDController(kP, kI, kD, encoderShooterMotor, NULL);
#endif
		shooterTimer = new Timer();
		shooterTimer->Start();

		rpm = target = 0;

		GetWatchdog().SetExpiration(0.5);
	}

	void Autonomous()
	{
		cout<<"############AutomousStart######################\n";
#ifdef VISIONSYSTEM
#ifdef CAMERA
		ColorImage *i;
		camera->GetImage(i);
		printf("\n\n\ncamera not null\n\n\n");
#endif
		visionSystem();
#endif
		cout<<"############AutomousEnd########################\n";
	} // autonomous

	/**
	 * This function is called once each time the robot enters operator control.
	 */
	void OperatorControl()
	{
#ifdef BOTDRIVEBUTTON
		enum
		{
			kTank, kArcade, kWheel
		} driveMode = kTank;
#endif
#if 0
#ifdef OPTICAL_SENSOR_TEST
		double oneTime = -1.0f;
		double twoTime = -1.0f;
#endif
#else
#ifdef OPTICAL_SENSOR_TEST
		OpticalSensorTimer->Start();
		bool oldFirstValue = false, oldSecondValue = false;
		double frisbeeOne, frisbeeTwo;
		frisbeeOne = frisbeeTwo = 0.0;
		// These are the times when the frisbee starts being seen and stops being seen
		// st(art|op)(1|2) are for sensors 1 and 2 respectively
		// st(art|op)3 is for the transition between the two sensors combined.
		// it is assumed that the sensors are less than one frisbee distance from each other
		double start1 = -1.0, start2 = -1.0, start3 = -1.0, stop1 = -1.0,
		stop2 = -1.0, stop3 = -1.0;
		// difference in times between the frisbees
		double time1 = -1.0, time2 = -1.0, time3 = -1.0;
		// speeds of the frisbees
		double speed1 = -1.0, speed2 = -1.0, speed3 = -1.0;
#endif
#endif
#ifdef ENCODERS
		encoderShooterMotor -> Start();
		encoderShooterMotor->SetDistancePerPulse(60/128*(60/24));
#endif
#ifdef WPILIBPID
		//encoderShooterMotor->Start();
		pid->SetInputRange(-1.0,1.0);
		pid->SetOutputRange(-1.0,1.0);
		pid->SetPercentTolerance(0.10);
		encoderShooterMotor->SetPIDSourceParameter(Encoder::kRate);
		pid->Enable();
#endif
		int count=0;

#ifdef HOMEBREWPID
		PIDTimer->Start();
		PIDSampleTimer->Start();
#endif
#ifdef TIMERS
		DiscTimer->Start();
		RPMTimer->Start();
#endif
#ifdef SHIFTERS
		shift->Set(DoubleSolenoid::kOff);
#endif
#ifdef VISIONSYSTEM
		VisionTimer->Start();
		//int visioncount = 0;
#endif
#ifdef BOTDRIVEBUTTON 
		driveMode = kArcade;
		cout<<"Arcade Selected"<<endl;
#endif
		GetWatchdog().SetEnabled(true);

		while (IsOperatorControl())
		{
			// give cRIO time for housekeeping
			Wait(0.005f);
			count++;
			GetWatchdog().Feed();

			// This controls whether the shooter is angled up or down
			if (ShooterStick->GetRawButton(SHOOTER_UP))
			{
				shooterAngleController->Set(DoubleSolenoid::kReverse);
			}
			if (ShooterStick->GetRawButton(SHOOTER_DOWN))
			{
				shooterAngleController->Set(DoubleSolenoid::kForward);
			}
#ifdef STATEMACHINE
			shooterController();
#else
			//setShooterMotors(-(1-ShooterStick->GetZ())/2);
			regulateMotors((int)(MAX_RPM * -(1-ShooterStick->GetZ())/2), true);
			if (ShooterStick->GetTrigger())
			loaderSolenoid->Set(DoubleSolenoid::kReverse);
			else
			loaderSolenoid->Set(DoubleSolenoid::kForward);

#endif
			// printf("%f\n",(float)DiscTimer->Get());
			dslcd->PrintfLine(DriverStationLCD::kUser_Line1, "Shooter RPM  %d",
					rpm);
			dslcd->PrintfLine(DriverStationLCD::kUser_Line2, "Target  RPM  %d",
					target);
			dslcd->UpdateLCD();
#ifdef TIMERS

			if (DriveStickL->GetRawButton(DISC_TIMER_RESET))
			{
				DiscTimer->Reset();
			}
#endif
			// cout<<FirstOpticalSensor->Get()<<"\n";
#ifdef ENCODERS
			if (ShooterStick->GetRawButton(PRINT_ENCODER))
			{
				cout<<encoderShooterMotor-> Get()<<endl;
			}
#endif

#ifdef OPTICAL_SENSOR_TEST
			// Do this so that if the value changes in the middle of execution, we still have the same value to compare
			frisbeeOne = FIRST_OPTICAL_SENSOR_VALUE;
			// if it toggles...
			if (frisbeeOne != oldFirstValue)
			{
				//printf("%f\n",(float)DiscTimer->Get());
				//cout<<DiscTimer->Get()<<endl;// "\n ";
				//cout << "sensor 1: " << FIRST_OPTICAL_SENSOR_VALUE << endl;
				if (frisbeeOne == 0)
				{
					stop1 = OpticalSensorTimer->Get();
					cout<<"Stop One "<<stop1<<endl;
					time1 = stop1 - start1;
					stop1 = start1 = -1;
					speed1 = FRISBEE_DIA/time1;
#ifdef FORTNIGHT
					printf(
							"The frisbee is moving at %f Furlongs per Fortnight as of Sensor1\n",
							CONVERT(speed1));
#else
					printf(
							"The frisbee is moving at %f Feet per Second as of Sensor1\n",
							CONVERT(speed1));
#endif
				}
				else //if (frisbeeOne == 1)

				{
					//OpticalSensorTimer->Start();
					start1 = OpticalSensorTimer->Get();
					cout<<"Start One "<<start1<<endl;
					start3 = OpticalSensorTimer->Get();
					cout<<"start three "<<start3<<endl;
				}
			} // f1 changed
			oldFirstValue = frisbeeOne;

			frisbeeTwo = SECOND_OPTICAL_SENSOR_VALUE;
			if (frisbeeTwo != oldSecondValue)
			{
				//cout << "sensor 2: " << SECOND_OPTICAL_SENSOR_VALUE << endl;
				//printf("%f\n",(float)DiscTimer->Get());
				//cout<<DiscTimer->Get()<<endl;// "\n ";
				if (frisbeeTwo == 0)
				{
					stop2 = OpticalSensorTimer->Get();
					cout<<"Stop Two "<<stop2<<endl;
					time2 = stop2 - start2;
					stop2 = start2 = -1;
					speed2 = FRISBEE_DIA/time2;
#ifdef FORTNIGHT
					printf(
							"The frisbee is moving at %f Furlongs per Fortnight as of Sensor2\n",
							CONVERT(speed2));
#else
					printf(
							"The frisbee is moving at %f Feet per Second as of Sensor2\n",
							CONVERT(speed2));
#endif
				}
				else //if (frisbeeTwo == 1)

				{
					//OpticalSensorTimer->Start();
					start2 = OpticalSensorTimer->Get();
					cout<<"Start two "<<start2<<endl;
					if (start3 != -1.0)
					{
						stop3 = OpticalSensorTimer->Get();
						cout<<"Stop 3 "<<stop3<<endl;
						time3 = stop3 - start3;
						stop3 = start3 = -1;
						speed3 = OPTICAL_SENSOR_DISTANCE/time3;
#ifdef FORTNIGHT
						printf(
								"The frisbee is moving at %f Furlongs per Fortnight as of Sensor1 and 2\n",
								CONVERT(speed3));
#else
						printf(
								"The frisbee is moving at %f Feet per Second as of Sensor1 and 2\n",
								CONVERT(speed3));
#endif
					}
				}
			} // f2 changed
			oldSecondValue = frisbeeTwo;

			/*if (!FIRST_OPTICAL_SENSOR_VALUE && !SECOND_OPTICAL_SENSOR_VALUE)
			 {
			 OpticalSensorTimer->Stop();
			 OpticalSensorTimer->Reset();
			 }*/
#endif
			//}
#ifdef BOTDRIVEBUTTON
			if (DriveStickL->GetRawButton(TANK_BUTTON)
					|| DriveStickR->GetRawButton(TANK_BUTTON))
			{
				driveMode = kTank;
				cout<<"Tank Selected"<<endl;
			}
			if (DriveStickL->GetRawButton(ARCADE_BUTTON)
					|| DriveStickR->GetRawButton(ARCADE_BUTTON))
			{
				driveMode = kArcade;
				cout << "Arcade Selected" << endl;
			}
			if (Wheel->GetRawButton(1) ||Wheel->GetRawButton(2)
					||Wheel->GetRawButton(3) || Wheel->GetRawButton(6))
			{
				driveMode = kWheel;
				cout<<"Wheel Selected"<<endl;
			}
			if (driveMode == kWheel)
			{
				drive->ArcadeDrive(DriveStickL->GetY(), Wheel->GetX());
				/*drive->SetLeftRightMotorOutputs(wheelDriveScale(Wheel->GetX(),
				 DriveStickL->GetY(), false), wheelDriveScale(
				 Wheel->GetX(), DriveStickL->GetY(), true));
				 //cout<<Wheel->GetX()<<endl;
				 if (Wheel->GetRawButton(4))
				 {
				 drive->SetLeftRightMotorOutputs(0.75f, -0.75f);
				 }
				 else if (Wheel->GetRawButton(5))
				 {
				 drive->SetLeftRightMotorOutputs(-0.75f, 0.75f);
				 }*/
			}
			if (driveMode == kArcade)
			{
				drive->ArcadeDrive(DriveStickL);
			}
			if (driveMode == kTank)
			{
				drive->TankDrive(DriveStickL, DriveStickR);
			}
			//assigns tank, arcade, or wheel mode to robot when you press buttons.;
#endif
#ifdef SHIFTERS
			if (DriveStickL->GetRawButton(SHIFT_UP_BUTTON)
					|| DriveStickR->GetRawButton(SHIFT_UP_BUTTON))
			{
				shift->Set(DoubleSolenoid::kForward);
			}
			else if (DriveStickL->GetRawButton(SHIFT_DOWN_BUTTON)
					|| DriveStickR->GetRawButton(SHIFT_DOWN_BUTTON))
			{
				shift->Set(DoubleSolenoid::kReverse);
			}
			else
			{
				shift->Set(DoubleSolenoid::kOff);
			}
#endif
#ifdef PRINT
#if 0
			if (count >= PRINTRATE)
			{
				if (encoderShooterMotor->Get() >= 128)
				{
					double time = RPMTimer->Get();
					int ticks = encoderShooterMotor->Get();
					printf("Shooter Encoder RPM: %lf\n", ((ticks /time))*60/128);
					printf("Shooter Motor RPM: %lf\n", ((ticks /time))*60/128*(40
									/24));
					printf("Shooter Wheel RPM: %lf\n", ((ticks /time))*60/128*(60
									/24));
					RPMTimer->Reset();
					encoderShooterMotor->Reset();
				}}
#endif
#endif

#ifdef WPILIBPID
			pid->SetPID(kP,kI,kD);
			pid->SetSetpoint(-(1-DriveStickL->GetThrottle())/2);

			float p = pid->Get();
			//dslcd->Printf(DriverStationLCD::kUser_Line1, 1, "PID(.8)=%1.4f", p);
			printf("pid=%1.4f\n", p);
			printf("pid=%1.4f\n", ShooterMotorLeft->Get());
			ShooterMotorLeft->Set(pid->Get());
			ShooterMotorRight->Set(ShooterMotorLeft->Get());
			//dslcd->Printf(DriverStationLCD::kUser_Line2, 1, "Motor RPM=%6f",
			//		getShooterWheelRPM());
#endif
#ifndef HOMEBREWPID
			setShooterMotors((DriveStickL->GetThrottle()-1)/2);
#endif
			//printf("Motor Rate=%7lf\n", encoderShooterMotor->GetRate());
#if defined(PRINT) && 0 
			if (count >= PRINTRATE)
			{
				printf("Wheel Y: %f\n", Wheel->GetX());
				int i;
				for (i = 1; i < 7; i++)
				printf("Button %d = %1d ", i, Wheel->GetRawButton(i));
			}
#endif
			if (count++ >= PRINTRATE)
			{
				count = 0;
			}
		} // while is operator control
	} // OperatorControl()

	void setShooterMotors(float desiredspeed)
	{
		ShooterMotorLeft->Set(desiredspeed);
		ShooterMotorRight->Set(desiredspeed);
	}
	int getShooterWheelRPM()
	{
		double time = RPMTimer->Get() - prevTime;
		int ticks = encoderShooterMotor->Get() - prevEnc;
		prevEnc = encoderShooterMotor->Get();
		prevTime = RPMTimer->Get();
		return (int)fabs((ticks / time)*60.0/128.0*60.0/24.0);
	}
#ifdef HOMEBREWPID
	float PID(float setpoint)
	{
		setpoint /= MAX_RPM;
		float preverror = error;
		//float current = encoderShooterMotor->GetRate() / MAX_RPM;
		rpm = getShooterWheelRPM();
		float current = rpm / MAX_RPM;
		float dt = PIDTimer->Get();
		PIDTimer->Reset();
		error = setpoint - current;
		accumerror += error * dt;
		P = kP * error;
		I = kI * accumerror;
		D = kD * (error-preverror) / dt;
		F = kF * setpoint;
		float out = P + I + D + F;
		//printf("P: %f, I: %f, D: %f\n", P, I, D);
		//printf("Error: %f\n", error);
		//printf("out: %f\n", out);
		if (out> 1.0)
			out = 1.0;
		if (out < -1)
			out = -1;
		return out;
	}
	float regulateMotors(int RPM, bool setMotors)
	{
		RPM = abs(RPM);
		target = RPM;
		kP = ds->GetAnalogIn(1);
		kI = ds->GetAnalogIn(2);
		kD = ds->GetAnalogIn(3);
		sampleRate = ds->GetAnalogIn(4);
		// make samplerate on a scale from 0-1
		sampleRate = sampleRate / 5;
		// makes sampleRate in range .001 - 1.0 
		sampleRate = sampleRate * (.001 + (1.0 - .001));

		if (PIDSampleTimer->Get() > sampleRate)
		{
			PIDSampleTimer->Reset();
			printCount++;
			oldp = p;
			p = -PID(RPM);
#if defined(PRINT) && 0
			if (oldp/p < 0)
			printf("switch\n");
			//if (fabs((int)(p*100 - oldp*100)) < 10 || oldp == 0 || p == 0) {
			/*if (p - oldp > .300)
			 {
			 p = oldp + .300;
			 }
			 if (p - oldp < -.300)
			 }
			 p = oldp - .300;
			 }*/
#endif
#if defined(PRINT) && defined(HOMEBREWPID) && 1
			if ((printCount % 1) == 0)
			{
#if 0
				printf("Target RPM: %i\nTarget: %f%%", RPM, RPM/MAX_RPM*100.0);
				printf("\nPID=%1.4f\n", p);
				printf("P: %f, I: %f, D: %f\n", P, I, D);
				printf("Error: %f\n", error);
				printf("\nMotor RPM =%7lf\n", (double)rpm);
				printf("Motor Speed: %f\n", ShooterMotorLeft->Get());
#else
				printGraphs();
#endif
			}
			float out = p + ShooterMotorLeft->Get();
			if (setMotors)
			{
				setShooterMotors(out);
			}
			return out;
		}
		return ShooterMotorLeft->Get();
#endif
	}
#endif
#if defined(PRINT) && defined(HOMEBREWPID) && 1
	void printGraphs()
	{
		//cout<<"Interval:"<<sampleRate<<"\t";
		initializePrintBuffers();
		int placeOfTarget = (int)((PRINT_BUFFER_SIZE-1) * target / MAX_RPM);

		// Why is this comment here?
		//  // _ becuase there are two different rpm's
		int placeOfrpm = (int)((PRINT_BUFFER_SIZE - 1) * getShooterWheelRPM()
				/ MAX_RPM + .5);

		int placeOfPWM = (int)((PRINT_BUFFER_SIZE - 1)
				* fabs(GET_MOTOR_SPEED) + .5);

		int placeOfP = (int)((PRINT_BUFFER_SIZE - 1) * kP / kPID_RANGE);
			int placeOfI = (int)((PRINT_BUFFER_SIZE - 1) * kI / kPID_RANGE);
		int placeOfD = (int)((PRINT_BUFFER_SIZE - 1) * kD / kPID_RANGE);
			
		
		printNumberInBuffer(placeOfTarget, 'T', 0);
		printNumberInBuffer(placeOfrpm, 'R', 0);
		printNumberInBuffer(placeOfPWM, 'W', 0);

		printNumberInBuffer(placeOfP, 'P', 1);
		printNumberInBuffer(placeOfI, 'I', 1);
		printNumberInBuffer(placeOfD, 'D', 1);

		printf("|%s|\n", printbuffer[0]);
		printf("|%s|\n", printbuffer[1]);
		printf("\n");

#if 0
		//Cleanup
		if(placeOfTarget <= PRINT_BUFFER_SIZE && placeOfTarget >= 0 )
		{
			printbuffer[placeOfTarget] = ' ';
		}
		else if(placeOfTarget> PRINT_BUFFER_SIZE)
		{
			printbuffer[PRINT_BUFFER_SIZE] = ' ';
		}
		else
		{
			printbuffer[0] = ' ';
		}
		if(placeOfrpm <= PRINT_BUFFER_SIZE && placeOfrpm >= 0)
		{
			printbuffer[placeOfrpm] = ' ';
		}
		else if(placeOfrpm> PRINT_BUFFER_SIZE)
		{
			printbuffer[PRINT_BUFFER_SIZE] = ' ';
		}
		else
		{
			printbuffer[0] = ' ';
		}
#else 
		int q = 0;
		for (; q < PRINT_BUFFER_SIZE; q++)
		{
			printbuffer[0][q] = ' ';
			printbuffer[1][q] = ' ';
		}
#endif
	}
	void printNumberInBuffer(int value, char character, int index)
	{
		if ((value <= (PRINT_BUFFER_SIZE-1)) && (value >= 0 ))
		{
			printbuffer[index][value] = character;
		}
		// it's out of bounds to the left or right, draw a '?'
		else if (value > (PRINT_BUFFER_SIZE-1))
		{
			// Print the lower case of the letter if it is out of bounds
			printbuffer[index][PRINT_BUFFER_SIZE-1] = character + ('a' - 'A');
		}
		else
		{
			printbuffer[index][0] = character + ('a' - 'A');
		}
	}
	void initializePrintBuffers()
	{
		int printindex;
		for (printindex=0; printindex<PRINT_BUFFER_SIZE; printindex++)
		{
			printbuffer[0][printindex]=' ';
			printbuffer[1][printindex]=' ';
		}

		printbuffer[0][PRINT_BUFFER_SIZE]='\0';
		printbuffer[1][PRINT_BUFFER_SIZE]='\0';

		/*printbuffer[0][0] = '|';
		printbuffer[1][0] = '|';
		printbuffer[0][PRINT_BUFFER_SIZE-1] = '|';
		printbuffer[1][PRINT_BUFFER_SIZE-1] = '|';*/
	}
#endif
#ifdef STATEMACHINE
	void shooterController()
	{
		/* compare entry and exit states, detect when state has changed */
		shooterState oldState;
		//static unsigned int stateChangeCount=0;
		static int remainingFrisbees = 4;
		static shooterState shootingState = start;
		static shooterMode shootingMode = noMode;
		static double shooterMotorSpinupSpeed;
		oldState = shootingState;
		if (ShooterStick->GetRawButton(START_STATE_MACHINE ))
		{
			shootingState = start;
		}
		switch (shootingState)
		{
		default:
			shootingState = start;
			break;

		case start:
			shootingState = idle;
			setShooterMotors(MOTOR_STOP);
			// retract cylinder, just in case.
			loaderSolenoid->Set(DoubleSolenoid::kForward); //TODO reverse?
			break;

		case idle:
			//cout<<"Idling"<<endl;
			// stop the motor when we aren't shooting
			setShooterMotors(MOTOR_STOP);
			if (ShooterStick->GetRawButton(STATE_MACHINE_MANUAL))
			{
				shootingState = setManualShoot;
			}
			if (ShooterStick->GetRawButton(STATE_MACHINE_DUMPa)
					|| ShooterStick->GetRawButton(STATE_MACHINE_DUMPb))
			{
				shootingState = setDumpShoot;
			}
			break;

		case setManualShoot:
			shootingMode = manualShoot;
			shootingState = startShooterMotor;
			shooterMotorSpinupSpeed = regulateMotors((int)(MAX_RPM*-(1-ShooterStick->GetZ())/2), false);
			break;

		case setDumpShoot:
			shootingMode = dump;
			shootingState = startShooterMotor;
			break;

		case shooterCheckTrigger:
			//Buttton One is Trigger
			//cout<<"checking trigger"<<endl;
			regulateMotors((int)(MAX_RPM*-(1-ShooterStick->GetZ())/2), true);
			if (ShooterStick->GetTrigger())
			{
				shootingState = extendCylinder;
			}
			/*else {
			 shootingState = idle;
			 }*/
			break;

		case startShooterMotor:
			regulateMotors((int)DUMP_RPM, true);
			if (shootingMode == dump)
			{
				shooterTimer->Reset();
				shootingState = shooterMotorSpinupWait;
			}

			/* for manual shooting, the operator has to set
			 ** the shooter motor speed manually. we
			 ** expect operator to wait for spinup before
			 ** pulling trigger:
			 * */

			if (shootingMode == manualShoot)
			{
				shootingState = shooterCheckTrigger;
			}
			break;

		case shooterMotorSpinupWait:
			if (shooterTimer->Get() >= SHOOTER_MOTOR_SPINUP_WAIT)
			{
				shootingState = extendCylinder;
			}
			break;

		case extendCylinder:
			//cout<<"firing"<<endl;
			loaderSolenoid->Set(DoubleSolenoid::kReverse);//TODO forward?
			shooterTimer->Reset();
			shootingState = extendCylinderWait;
			break;

		case extendCylinderWait:
			if (shooterTimer->Get() >= SHOOTER_CYLINDER_EXTEND_TIME)
			{
				shooterTimer->Reset();
				shootingState=retractCylinder;
			}
			break;

		case retractCylinder:
			loaderSolenoid->Set(DoubleSolenoid::kForward);//TODO reverse?
			shooterTimer->Reset();
			shootingState = retractCylinderWait;
			break;

		case retractCylinderWait:
			if (shooterTimer->Get() >= SHOOTER_CYLINDER_RETRACT_TIME)
			{
				shooterTimer->Reset();
				if (shootingMode == dump)
				{
					shootingState = checkFrisbeeCount;
				}
				if (shootingMode == manualShoot)
				{
					shootingState = shooterCheckTrigger;
				}
			}
			break;

		case checkFrisbeeCount:
			--remainingFrisbees;
			//cout<<"remaing frisbees"<<remainingFrisbees<<endl;
			if (remainingFrisbees <= 0)
			{
				remainingFrisbees = 4;
				shootingState = stopShooterMotor;
			}
			else
			{
				shootingState = extendCylinder;
			}
			break;
		case stopShooterMotor:
			//cout<<"stopping";
			setShooterMotors(MOTOR_STOP);
			shootingState = idle;
			break;

		}
		// switch shootingState
#if 0
		stateChangeCount++;

		if (oldState != shootingState)
		{ // state has changed. print old,new,count
			printf("ShooterState: old, new, cnt: %2u %2u %6u\n",
					(unsigned int)oldState,
					(unsigned int)shootingState,
					stateChangeCount);

			stateChangeCount=0;

		}
#endif
	}// end shooterController
#endif
#ifdef VISIONSYSTEM
	/**
	 * Computes the estimated distance to a target using the height of the particle in the image. For more information and graphics
	 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
	 * 
	 * @param image The image to use for measuring the particle estimated rectangle
	 * @param report The Particle Analysis Report for the particle
	 * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
	 * @return The estimated distance to the target in Inches.
	 */

	void visionSystem(void)
	{
		Threshold threshold(60, 100, 90, 255, 20, 255); //HSV threshold criteria, ranges are in that order ie. Hue is 60-100 ParticleFilterCriteria2
		ParticleFilterCriteria2 criteria[] =
		{
		{ IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false } };
		VisionTimer->Start();
		ColorImage *image;
		Scores *scores;
		//image = new RGBImage("/testImage.jpg");	// get the sample image from the cRIO flash
#ifdef CAMERA 
		camera->GetImage(image); //To get the images from the camera comment the line above and uncomment this one
#else
		image = new RGBImage("/HybridLine_SmallGreen4.jpg");
#endif	

		BinaryImage *thresholdImage = image->ThresholdHSV(threshold); // get just the green target pixels
		//thresholdImage->Write("/threshold.bmp");
		BinaryImage *convexHullImage = thresholdImage->ConvexHull(false); // fill in partial and full rectangles
		//convexHullImage->Write("/ConvexHull.bmp");
		BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria,
				1); //Remove small particles
		//filteredImage->Write("Filtered.bmp");

		vector<ParticleAnalysisReport> *reports =
				filteredImage->GetOrderedParticleAnalysisReports(); //get a particle analysis report for each particle

		scores = new Scores[reports->size()];

		//Iterate through each particle, scoring it and determining whether it is a target or not
		for (unsigned i = 0; i < reports->size(); i++)
		{
			ParticleAnalysisReport *report = &(reports->at(i));

			scores[i].rectangularity = scoreRectangularity(report);
			scores[i].aspectRatioOuter = scoreAspectRatio(filteredImage,
					report, true);
			scores[i].aspectRatioInner = scoreAspectRatio(filteredImage,
					report, false);
			scores[i].xEdge = scoreXEdge(thresholdImage, report);
			scores[i].yEdge = scoreYEdge(thresholdImage, report);

			if (scoreCompare(scores[i], false))
			{
				printf(
						"particle: %d  is a High Goal  centerX: %f  centerY: %f \n",
						i, report->center_mass_x_normalized,
						report->center_mass_y_normalized);
				printf("Distance: %f \n", computeDistance(thresholdImage,
						report, false));
				finalreports[i]->report = *report;
				finalreports[i]->type = TOP_GOAL;
			}
			else if (scoreCompare(scores[i], true))
			{
				printf(
						"particle: %d  is a Middle Goal  centerX: %f  centerY: %f \n",
						i, report->center_mass_x_normalized,
						report->center_mass_y_normalized);
				printf("Distance: %f \n", computeDistance(thresholdImage,
						report, true));
				finalreports[i]->report = *report;
				finalreports[i]->type = MID_GOAL;
			}
			else
			{
				printf(
						"particle: %d  is not a goal  centerX: %f  centerY: %f \n",
						i, report->center_mass_x_normalized,
						report->center_mass_y_normalized);
				finalreports[i]->report = *report;
				finalreports[i]->type = NON_GOAL;
			}
			printf("rect: %f  ARinner: %f \n", scores[i].rectangularity,
					scores[i].aspectRatioInner);
			printf("ARouter: %f  xEdge: %f  yEdge: %f  \n",
					scores[i].aspectRatioOuter, scores[i].xEdge,
					scores[i].yEdge);
		}
		printf("\n");

		// be sure to delete images after using them
		delete filteredImage;
		delete convexHullImage;
		delete thresholdImage;
		delete image;

		//delete allocated reports and Scores objects also
		delete scores;
		delete reports;

		cout<<"\n *****Vision Time:"<<VisionTimer->Get()<<"\n";
		VisionTimer->Reset();
	}
	double computeDistance(BinaryImage *image, ParticleAnalysisReport *report,
			bool outer)
	{
		double rectShort, height;
		int targetHeight;

		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0,
				IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
		//using the smaller of the estimated rectangle short side and the bounding rectangle height results in better performance
		//on skewed rectangles
		height = min(report->boundingRect.height, rectShort);
		targetHeight = outer ? 29 : 21;
		return X_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
	}

	/**
	 * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
	 * the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed by moving
	 * to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle area= x*y
	 * and particle perimeter= 2x+2y
	 * 
	 * @param image The image containing the particle to score, needed to perform additional measurements
	 * @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
	 * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio for the inner target or the outer
	 * @return The aspect ratio score (0-100)
	 */
	double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report,
			bool outer)
	{
		double rectLong, rectShort, idealAspectRatio, aspectRatio;
		idealAspectRatio = outer ? (62/29) : (62/20); //Dimensions of goal opening + 4 inches on all 4 sides for reflective tape

		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0,
				IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0,
				IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);

		//Divide width by height to measure aspect ratio
		if (report->boundingRect.width> report->boundingRect.height)
		{
			//particle is wider than it is tall, divide long by short
			aspectRatio = 100*(1 -fabs((1 -((rectLong/rectShort)
					/idealAspectRatio))));
		}
		else
		{
			//particle is taller than it is wide, divide short by long
			aspectRatio = 100*(1 -fabs((1 -((rectShort/rectLong)
					/idealAspectRatio))));
		}
		return (max(0, min(aspectRatio, 100))); //force to be in range 0-100
	} //ScroreAspectRatio {}

	/**
	 * Compares scores to defined limits and returns true if the particle appears to be a target
	 * 
	 * @param scores The structure containing the scores to compare
	 * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
	 * 
	 * @return True if the particle meets all limits, false otherwise
	 */

	bool scoreCompare(Scores scores, bool outer)
	{
		bool isTarget = true;

		isTarget &= scores.rectangularity> RECTANGULARITY_LIMIT;
		if (outer)
		{
			isTarget &= scores.aspectRatioOuter> ASPECT_RATIO_LIMIT;
		}
		else
		{
			isTarget &= scores.aspectRatioInner> ASPECT_RATIO_LIMIT;
		}
		isTarget &= scores.xEdge> X_EDGE_LIMIT;
		isTarget &= scores.yEdge> Y_EDGE_LIMIT;

		return isTarget;
	}

	/**
	 * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
	 * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
	 * 
	 * @param report The Particle Analysis Report for the particle to score
	 * @return The rectangularity score (0-100)
	 */
	double scoreRectangularity(ParticleAnalysisReport *report)
	{
		if (report->boundingRect.width*report->boundingRect.height !=0)
		{
			return 100*report->particleArea/(report->boundingRect.width
					*report->boundingRect.height);
		}
		else
		{
			return 0;
		}
	}

	/**
	 * Computes a score based on the match between a template profile and the particle profile in the X direction. This method uses the
	 * the column averages and the profile defined at the top of the sample to look for the solid vertical edges with
	 * a hollow center.
	 * 
	 * @param image The image to use, should be the image before the convex hull is performed
	 * @param report The Particle Analysis Report for the particle
	 * 
	 * @return The X Edge Score (0-100)
	 */
	double scoreXEdge(BinaryImage *image, ParticleAnalysisReport *report)
	{
		double total = 0;
		LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(),
				IMAQ_COLUMN_AVERAGES, report->boundingRect);
		for (int i=0; i < (averages->columnCount); i++)
		{
			if (xMin[i*(XMINSIZE-1)/averages->columnCount]
					< averages->columnAverages[i]
					&& averages->columnAverages[i] < xMax[i*(XMAXSIZE-1)/averages->columnCount])
			{
				total++;
			}
		}
		total = 100*total/(averages->columnCount); //convert to score 0-100
		imaqDispose(averages); //let IMAQ dispose of the averages struct
		return total;
	}

	/**
	 * Computes a score based on the match between a template profile and the particle profile in the Y direction. This method uses the
	 * the row averages and the profile defined at the top of the sample to look for the solid horizontal edges with
	 * a hollow center
	 * 
	 * @param image The image to use, should be the image before the convex hull is performed
	 * @param report The Particle Analysis Report for the particle
	 * 
	 * @return The Y Edge score (0-100)
	 */
	double scoreYEdge(BinaryImage *image, ParticleAnalysisReport *report)
	{
		double total = 0;
		LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(),
				IMAQ_ROW_AVERAGES, report->boundingRect);
		for (int i=0; i < (averages->rowCount); i++)
		{
			if (yMin[i*(YMINSIZE-1)/averages->rowCount] < averages->rowAverages[i]
					&& averages->rowAverages[i] < yMax[i*(YMAXSIZE-1)/averages->rowCount])
			{
				total++;
			}
		}
		total = 100*total/(averages->rowCount); //convert to score 0-100
		imaqDispose(averages); //let IMAQ dispose of the averages struct
		return total;
	}
#endif
};
START_ROBOT_CLASS(RobotDemo)
;

#if 0

20130124: *****Robot drive tested, but not driven on ground. DONE
Encoders questionable.
Driven on 6.68.9 and 6.68.6
Need to put a camera code and targeting.
Controller Setup: Arcade, Steering Wheel, Button Mapping
*****Need code to shift. DONE
*****Need code for pnuemadic pump. DONE
*****Design control system: joysticks, buttons. DONE
Regulate shooter motor speed.
Optical freezbie timer.

20130124: JLA & GK: Fixed Left/Right Back/Forward swap problem.
Reassigned joysticks to fix left/right swap
Deleted second moter invertion block in robot
loop to fix back/forward

20130128: JLA $ GK: Worked on Vision Control
Done& Added timers to vision sample timing the legnth
& of the vision processing
& NOTE***************************************
& *******************************************
& *******************************************
& Crio on dummy Bot (not kitbot) needs to be
& reflashed
& ENDNOTE************************************
& *******************************************
& *******************************************

20130129: HS & JLA: reimaged the crio on plywood robot.
vision processing takes 0.15 sec.
20130129: JLA & GK: Ported the VisionSample 2013 example to KitbotCode
Function visionSytem(void) calls vision code
Added
#define VISIONSYSTEM for Vision code
20130131: JLA & GK: Added code using joystick to set shooter motor speed
Done& NOTE***********************************************
& ***************************************************
& ***************************************************
& Bug in printing on termenal, unknown garbage output printing
& ENDNOTE********************************************
& ***************************************************
& ***************************************************
2012031: JLA & CJ & GK: Added compressor and soleniod code.
20130206: ML & CJ & GK: Added things to the vision system.
20130207: JLA: *****moved Solenoids from type Solenoids to type DoubleSolenoids to support movement in both directions. (NEEDS TESTING)
*****Altered shifting code to allow for movemnt in both directions. The Enum of the type required to chenge direction is under DoubleSolenoid::(NEEDS TESTING)
Changed PWM motor inputs to support the redesigned robot.
20130209: JLA: Rewrote shooter code according to discussed algorithems.
NOTE**************************************************
******************************************************
******************************************************
Code nonfunctional, I have not managed to get the code function
Numbers seems to have no rhyme and reason to it.
ENDNOTE***********************************************
******************************************************
******************************************************
201302012: JLA: worked on wheel code (all I knew how to do)
changed type of drive controll to a enum, hit a button on the wheel to activate wheel mode, 8 & 9 on drivestickl to turn on tank and arcade
201302014: JLA: Finished wheel code and the robot should be drivable with it now.
Tested the wheel code on real robot and it works.
20130215: JLA, Mark, GaryK:designed and started coding shooter state machine. It's INCOMPLETE.
Compile fails with syntax err where I (garyk) left off.
20130216: JLA, CJ: backed up all the programs into C:\Documents and Settings\Programming\My Documents\FRC2013\Backups.
Changed PWM's for shooter and solenoids.
20130321: JLA, CJ: Started a buffer writer to show the Goal, currrent position, and 0 of the PID code
NOT FINISHED

#endif
