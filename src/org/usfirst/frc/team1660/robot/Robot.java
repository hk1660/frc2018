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


public class Robot<m_robotDrive> extends IterativeRobot {

	/*----DECLARED GLOBAL VARIABLES-------*/

	AHRS ahrs;
	boolean autoDriveFlag = false;	//automatic driving

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
	double roboAngle = 0.0;
	double fieldAngleDifference = 0.0;
	double lastUsedAngle;

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
	/*
	//this changes the driving to be based on direct voltage input
	public void changeDrivingToVoltage(){
		this.frontLeft.changeControlMode(WPI_TalonSRX.TalonControlMode.Voltage);
		this.backLeft.changeControlMode(WPI_TalonSRX.TalonControlMode.Voltage);
		this.frontRight.changeControlMode(WPI_TalonSRX.TalonControlMode.Voltage);
		this.backRight.changeControlMode(WPI_TalonSRX.TalonControlMode.Voltage);
	}
	*/
	//This changes the driving to be based on percentage of voltage
	public void changeDrivingToPercent(){
		
		
		frontLeft.changeControlMode(WPI_TalonSRX.ControlMode.PercentOutput);
		backLeft.changeControlMode(WPI_TalonSRX.ControlMode.PercentOutput);
		frontRight.changeControlMode(WPI_TalonSRX.ControlMode.PercentOutput);
		backRight.changeControlMode(WPI_TalonSRX.ControlMode.PercentOutput);
		mecDrive.setMaxOutput(1.0);	
	}
	
	public double autoTurnSpeed (double futureAngle){
		changeDrivingToPercent();
		double actualangle = this.getCurrentAngle();
		double diff = futureAngle - actualangle;	//positive deg for right turns
		// make the diff-values range from  -179 to 179
		if (diff > 180){ 			//to handle extra large right turns
			diff = diff - 360;
		} else if (diff < -180){	//to handle extra large left turns
			diff = diff + 360;
			
		}
		return diff;
	}
		
	public void autoTurn(int futureAngle){

		//find correct speed to turn
		double desired_speed = autoTurnSpeed(futureAngle);

		double strafeSpeedLeft= driverStick.getRawAxis(LT_AXIS);
		double strafeSpeedRight = driverStick.getRawAxis(RT_AXIS);
		double strafeSpeedActual=0;
		double minMotorSpeed = .3;
		if(strafeSpeedLeft>0){
			strafeSpeedActual=((Math.pow(strafeSpeedLeft,2))/.3)+minMotorSpeed;
			strafeSpeedActual=strafeSpeedActual*-1;
		}
		else if (strafeSpeedRight>0){
			strafeSpeedActual=(Math.pow(strafeSpeedRight,2)/.3)+minMotorSpeed;
		}

		//keep turning until within the tolerance from desired angle
		mecDrive.driveCartesian(strafeSpeedActual, desired_speed, getCurrentAngle());
	}
	
	// method to turn robot to different angles automatically @aldenis @marlahna
	public void checkAutoTurn(){

		if(driverStick.getPOV()==this.POV_LEFT){
			autoDriveFlag = true;
			this.autoTurn(300);		//aiming at the Right Peg
			this.lastUsedAngle=300;

		}
		else if(driverStick.getPOV()==this.POV_DOWN){
			autoDriveFlag = true;
			this.autoTurn(180);	//heading back towards driverStation
			this.lastUsedAngle=180;

		}
		else if(driverStick.getPOV()==this.POV_RIGHT){
			autoDriveFlag = true;
			this.autoTurn(60);		//aiming at the Left Peg
			this.lastUsedAngle=60;

		}
		else if(driverStick.getPOV()==this.POV_UP){
			autoDriveFlag = true;
			this.autoTurn(0);	//heading away from driverStation
			this.lastUsedAngle=0;
		}
		else{
			autoDriveFlag=false;
		}
	}


*/

	/*------------------------- CUSTOM METHODS -------------------------*/

	/*----- JOYSTICK METHODS -----*/
	//method to check joysticks for driving the robot -Nana B & Matthew W
	public void checkDriving() {

		double strafe = squareIt(driverStick.getRawAxis(RIGHT_X_AXIS)) ; // right and left on the left thumb stick?
		double forward = squareIt(driverStick.getRawAxis(RIGHT_Y_AXIS));// up and down on left thumb stick?
		double turn = squareIt(driverStick.getRawAxis(LEFT_X_AXIS));// right and left on right thumb stick

		//we think the parameters were wrong: S>T>F not F>S>T
		mecDrive.driveCartesian(-strafe, -turn, -forward, getCurrentAngle());
		
	}

	//method to square the joystick values to provide less sensitivity for small movements -Matthew W
	public double squareIt(double joy) {

		double squared = joy * joy;

		
		//what happens if the value is negative? do we really want to make every joystick value positive?
		if(joy < 0) {
			squared *= -1;
		}
		
		return squared;
	}

	//joystick method to manually resets the robot to the field's orientation -Aldenis G
	public void checkResetAngle(){

		if (driverStick.getRawButton(START_BUTTON)) {
			resetAngle();
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



	/*----- AUTONOMOUS STRATEGY METHODS -----*/















}
