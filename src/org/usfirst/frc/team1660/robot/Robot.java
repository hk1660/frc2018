/*
 * Code for the Harlem Knights (FRC 1660) Robot for 2018
 * website: www.hk1660.com
 * online repository: www.github.com/hk1660/frc2018
 */

package org.usfirst.frc.team1660.robot;

/*-----IMPORTED LIBRARIES-----*/

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class Robot<m_robotDrive> extends IterativeRobot {

	/*----DECLARED GLOBAL VARIABLES-------*/

	//Drivetrain Declarations
	private static final int kFrontLeftChannel = 1;
	private static final int kBackLeftChannel = 2;
	private static final int kFrontRightChannel = 3;
	private static final int kBackRightChannel = 4;
	private WPI_TalonSRX frontLeft;
	private WPI_TalonSRX backLeft;
	private WPI_TalonSRX frontRight;
	private WPI_TalonSRX backRight;
	private MecanumDrive mecDrive;

	private AHRS navx;
	double fieldAngleDifference = 0.0;
	double roboAngle = 0.0;
	double lastUsedAngle;
	boolean autoDriveFlag = false;	//automatic driving


	//Manipulator Declarations
	private static final int kMouthMotorChannel = 4;
	private static final int kLiftMotorChannel = 5;	
	private static final int kSwitchBoi = 6; 			//lol we can switch the name l8r

	private WPI_TalonSRX liftMotor;
	private WPI_TalonSRX mouthMotor;

	//JOYSTICK - Nana B. & Mathew W.
	private Joystick driverStick = new Joystick(0);
	private Joystick manipStick = new Joystick(1);
	private final int A_BUTTON = 1;
	private final int B_BUTTON = 2;
	private final int X_BUTTON = 3;
	private final int Y_BUTTON = 4;
	private final int LB_BUTTON = 5;
	private final int RB_BUTTON = 6;
	private final int BACK_BUTTON = 7;
	private final int START_BUTTON = 8;
	private final int LEFT_JOY_BUTTON = 9;
	private final int RIGHT_JOY_BUTTON = 10;
	private final int LEFT_X_AXIS = 0;
	private final int LEFT_Y_AXIS = 1;
	private final int LT_AXIS = 2;
	private final int RT_AXIS = 3;
	private final int RIGHT_X_AXIS = 4;
	private final int RIGHT_Y_AXIS = 5;
	private final int POV_UP = 0;
	private final int POV_LEFT = 270;
	private final int POV_DOWN = 180;
	private final int POV_RIGHT = 90;


	/*----- REQUIRED FRC MAIN METHODS -----*/
	public void robotInit() {

		//Drivetrain Initializations
		frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
		backLeft = new WPI_TalonSRX(kBackLeftChannel);
		frontRight = new WPI_TalonSRX(kFrontRightChannel);
		backRight = new WPI_TalonSRX(kBackRightChannel);

		frontLeft.setInverted(true);	// Invert the left side motors (Is this needed?)
		backLeft.setInverted(true);		// Invert the left side motors (Is this needed?)

		//we think the constructor switched the 3rd & 4th parameters
		mecDrive = new MecanumDrive(frontLeft, backLeft, backRight, frontRight);
		
		//Manipulator Initializations
		mouthMotor = new WPI_TalonSRX(kMouthMotorChannel);
		liftMotor = new WPI_TalonSRX(kLiftMotorChannel); //A.K.A Elevator/Climb Manipulator

		//Sensor Intializations
		try {
			navx = new AHRS(SPI.Port.kMXP); //navX-MXP initialized with (SPI, I2C, TTL UART) and USB //http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}


	}

	//AUTONOMOUS MODE
	public void autonomousInit() {
		//autocode that is used for every strategy goes here @AmadouGamby & @marlahna
		// timer.reset(); // Resets the timer to 0
		//timer.start(); // Start counting
		resetAngle();
	}	
	public void autonomousPeriodic() {

	}

	//TELEOP MODE
	public void teleopInit() { 
		resetAngle();  //delete later
	}
	
	public void teleopPeriodic() {

		checkDriving();
		checkResetAngle();

		getCurrentAngle();
		getEncoder();

	}


	
	/*------------------------- CUSTOM METHODS -------------------------*/

	/*----- JOYSTICK METHODS -----*/
	//method to check joysticks for driving the robot -Nana B & Matthew W
	public void checkDriving() {

		double strafe = squareIt(driverStick.getRawAxis(RIGHT_X_AXIS)) ; // right and left on the left thumb stick?
		double forward = squareIt(driverStick.getRawAxis(RIGHT_Y_AXIS));// up and down on left thumb stick?
		double turn = squareIt(driverStick.getRawAxis(LEFT_X_AXIS));// right and left on right thumb stick

		
		//MECANUM -Malachi P
		if(autoDriveFlag == false ){

			//we think the parameters were wrong: S>T>F not F>S>T
			mecDrive.driveCartesian(-strafe, -turn, -forward, getCurrentAngle());

			//Prints
			SmartDashboard.putNumber("move",	forward);
			SmartDashboard.putNumber("rotate",	turn);
			SmartDashboard.putNumber("strafe",	strafe);
		} 

		SmartDashboard.putBoolean("AutoDriving?", autoDriveFlag);
		
	}

	//method to square the joystick values to provide less sensitivity for small movements -Matthew W
	public double squareIt(double joy) {

		double squared = joy * joy;
		
		//what happens if the value is negative? do we really want to make every joystick value positive?
		if(joy < 0) { squared *= -1; }
	
		return squared;
	}

	//joystick method to manually resets the robot to the field's orientation -Aldenis G
	public void checkResetAngle(){

		if (driverStick.getRawButton(START_BUTTON)) {
			resetAngle();
		}
	}

	// method to turn robot to different angles automatically @aldenis @marlahna
		public void checkAutoTurn(){

			if(driverStick.getPOV()==this.POV_LEFT){
				autoDriveFlag = true;
				autoTurn(270);		//aiming to the RIGHT
				this.lastUsedAngle=270;

			}
			else if(driverStick.getPOV()==this.POV_DOWN){
				autoDriveFlag = true;
				autoTurn(180);	//heading back towards driverStation
				this.lastUsedAngle=180;

			}
			else if(driverStick.getPOV()==this.POV_RIGHT){
				autoDriveFlag = true;
				autoTurn(90);		//aiming to the RIGHT
				this.lastUsedAngle=90;

			}
			else if(driverStick.getPOV()==this.POV_UP){
				autoDriveFlag = true;
				autoTurn(0);	//heading away from driverStation
				this.lastUsedAngle=0;
			}
			else{
				autoDriveFlag=false;
			}
		}
	
		

		

	/*----- SENSOR METHODS -----*/

	//method to update the angle the robot is facing on the field -Aldenis G
	public int getCurrentAngle(){

		int rawAngle = Math.floorMod((int)navx.getAngle(), 360);
		fieldAngleDifference = Math.floorMod((int)fieldAngleDifference, 360);
		int fieldAngle = Math.floorMod((rawAngle -(int) fieldAngleDifference), 360);

		SmartDashboard.putNumber("rawAngle", rawAngle);
		SmartDashboard.putNumber("fieldAngle", fieldAngle);
		System.out.println("rawAngle:"+rawAngle);
		System.out.println("fieldAngle:"+fieldAngle);

		return fieldAngle;
	}

	public void resetAngle() {
		fieldAngleDifference = navx.getAngle();
	}

	//method to get the value from the armabot encoder- lakiera and pinzon
	public int getEncoder(){

		int x = liftMotor.getSelectedSensorPosition(0);
		SmartDashboard.putNumber("encoderPosition", x);
		return x;
	}


	/*----- BASIC ROBOT MOTION METHODS -----*/

	//Basic method to climb down -Aldenis
	public void climbDown() {
		liftMotor.set(-1.0);
	}

	// Basic method for robot to spit out a PowerCube -Kwaku Boafo
	public void spit(){
		mouthMotor.set(1.0);
	}


	/*-----AUTO FUNCTION METHODS -----*/
	
	public double autoTurnSpeed (double futureAngle){
		//changeDrivingToPercent();  //do this manually with each "set" command
		double actualangle = this.getCurrentAngle();
		double diff = futureAngle - actualangle;	//positive deg for right turns
		// make the diff-values range from  -179 to 179
		if (diff > 180){ 			//to handle extra large right turns
			diff = diff - 360;
		} else if (diff < -180){	//to handle extra large left turns
			diff = diff + 360;
		}
		SmartDashboard.putNumber("AutoTurn Diff", diff);

		double angle_tolerance = 5.0;
		double min_speed = 0.4;
		double desired_speed;

		//adjust speeds to decrease as you approach the desired angle
		desired_speed = (1.0-min_speed) * (Math.abs(diff)/180) + min_speed;

		// assigning speed based on positive or negative - Kahlil & Malachi P
		if(diff > angle_tolerance ){  //right hand turn - speed
			desired_speed = -desired_speed;
		} else if(diff < -angle_tolerance){ // left hand turn +speed
			desired_speed = desired_speed;		
		}else{
			desired_speed = 0.0;
		}

		SmartDashboard.putNumber("AutoTurn Speed", desired_speed);
		System.out.println("ANGLE: "+ actualangle + " DIFF IS: " + diff + " DESIRED SPEED IS " + desired_speed);
	
		return desired_speed;	
	}
		
	public void autoTurn(int futureAngle){

		//find correct speed to turn
		double desired_speed = autoTurnSpeed(futureAngle);

		//Ability to Strafe while maintaining the desired angle
		double strafeSpeedLeft= driverStick.getRawAxis(LT_AXIS);
		double strafeSpeedRight = driverStick.getRawAxis(RT_AXIS);
		double strafeSpeedActual;
		double minMotorSpeed = 0.3;
		if(strafeSpeedLeft>0) {
			strafeSpeedActual=-(((Math.pow(strafeSpeedLeft,2))/0.3)+minMotorSpeed);
		} else if (strafeSpeedRight>0){
			strafeSpeedActual=(Math.pow(strafeSpeedRight,2)/0.3)+minMotorSpeed;
		} else {
			strafeSpeedActual = 0.0;
		}

		//keep turning until within the tolerance from desired angle
		mecDrive.driveCartesian(strafeSpeedActual, desired_speed, 0.0, 0.0);
	}


	/*----- AUTONOMOUS STRATEGY METHODS -----*/















}
