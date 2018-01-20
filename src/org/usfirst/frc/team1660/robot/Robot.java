/*
 * Code for the Harlem Knights (FRC 1660) Robot for 2018
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
	
	//Drivetrain Declarations
	private static final int kFrontLeftChannel = 2;
	private static final int kBackLeftChannel = 3;
	private static final int kFrontRightChannel = 1;
	private static final int kBackRightChannel = 0;
	WPI_TalonSRX frontLeft;
	WPI_TalonSRX backLeft;
	WPI_TalonSRX frontRight;
	WPI_TalonSRX backRight;
	private MecanumDrive mecDrive;
	
	private AHRS navx;
	double roboAngle = 0.0;
	double fieldAngleDifference = 0.0;

	//Manipulator Declarations
	private static final int kMouthMotorChannel = 4;
	private static final int kLiftMotorChannel = 5;	
	private static final int kSwitchBoi = 6; 			//lol we can switch the name l8r

	WPI_TalonSRX liftMotor;
	WPI_TalonSRX mouthMotor;

	//JOYSTICK - Nana B. & Mathew W.
	Joystick driverStick = new Joystick(0);
	Joystick manipStick = new Joystick(1);
	final int A_BUTTON = 1;
	final int B_BUTTON = 2;
	final int X_BUTTON = 3;
	final int Y_BUTTON = 4;
	final int LB_BUTTON = 5;
	final int RB_BUTTON = 6;
	final int BACK_BUTTON = 7;
	final int START_BUTTON = 8;
	final int LEFT_JOY_BUTTON = 9;
	final int RIGHT_JOY_BUTTON = 10;
	final int LEFT_X_AXIS = 0;
	final int LEFT_Y_AXIS = 1;
	final int LT_AXIS = 2;
	final int RT_AXIS = 3;
	final int RIGHT_X_AXIS = 4;
	final int RIGHT_Y_AXIS = 5;
	final int POV_UP = 0;
	final int POV_LEFT = 270;
	final int POV_DOWN = 180;
	final int POV_RIGHT = 90;
	
	final int FORWARDBACKWARD_AXIS = LEFT_Y_AXIS; //Left joystick up and down
	final int TURNSIDEWAYS_AXIS = RIGHT_X_AXIS; //Right joystick side to side
	final int STRAFE_AXIS = LEFT_X_AXIS; //Left joystick side to side

	public double squareInput(double x) {
		if(x > 0 ) {
			return Math.pow(x, 4);
		}
		else{
			return -1*Math.pow(x, 4); 
		}
	}
	
    public void robotInit() {

    	//Drivetrain Initializations
		frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
		backLeft = new WPI_TalonSRX(kBackLeftChannel);
		frontRight = new WPI_TalonSRX(kFrontRightChannel);
		backRight = new WPI_TalonSRX(kBackRightChannel);
		
		frontLeft.setInverted(true);	// Invert the left side motors (Is this needed?)
		backLeft.setInverted(true);		// Invert the left side motors (Is this needed?)

		mecDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

		
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


	/*----- AUTONOMOUS MODE -----*/
	public void autonomousInit() {
		//autocode goes here @AmadouGamby & @marlahna
		// timer.reset(); // Resets the timer to 0
	     //timer.start(); // Start counting
	}	
	public void autonomousPeriodic() {

	}

	/*----- TELEOP MODE -----*/ 
	public void teleopInit() { 
		
	}
	public void teleopPeriodic() {
		
		checkDriving();
		checkResetAngle();
		
		getCurrentAngle();
		getEncoder();
		
	}




	/*----- CUSTOM METHODS -----*/

	public void checkDriving() {

		double strafe = squareIt(driverStick.getRawAxis(RIGHT_X_AXIS)) ; // right and left on the left thumb stick?
		double forward = squareIt(driverStick.getRawAxis(RIGHT_Y_AXIS));// up and down on left thumb stick?
		double turn = squareIt(driverStick.getRawAxis(LEFT_X_AXIS));// right and left on right thumb stick

		mecDrive.driveCartesian(forward, strafe, turn);
	}
	
	public double squareIt(double joy) {
		double squared = joy * joy;
	//SmartDashboard.putNumber(key, value)
		return squared;
	}
	
	public int getCurrentAngle(){
		int rawAngle = Math.floorMod((int)navx.getAngle(), 360);
		fieldAngleDifference = Math.floorMod((int)fieldAngleDifference, 360);
		int fieldAngle = Math.floorMod((rawAngle -(int) fieldAngleDifference), 360);

		//float rawAngle = navx.getYaw();
		//float moddedfieldAngleDifference = navx.get
		
		SmartDashboard.putNumber("rawAngle", rawAngle);
		//SmartDashboard.putNumber("moddedfieldAngleDifference", moddedfieldAngleDifference);
		SmartDashboard.putNumber("fieldAngle", fieldAngle);
		
		System.out.println("rawAngle:"+rawAngle);
		//System.out.println("moddedfieldAngleDifference:"+moddedfieldAngleDifference);
		System.out.println("fieldAngle:"+fieldAngle);
		return fieldAngle;

	}

	public void checkResetAngle(){
		if (driverStick.getRawButton(START_BUTTON)) {
			fieldAngleDifference = navx.getAngle();		
		}
	}
	
	//Basic method to climb down -Aldenis
	public void climbDown() {
		liftMotor.set(-1.0);
		
	}
	

	
	
	// Basic method for robot to spit out a PowerCube -Kwaku Boafo
	public void spit(){
		mouthMotor.set(1.0);
	}

	

	//method to get the value from the armabot encoder- lakiera and pinzon
	public int getEncoder(){
		
		int x = liftMotor.getSelectedSensorPosition(0);
		SmartDashboard.putNumber("encoderPosition", x);
		return x;
	}	
	
	
	
	
	
	
	
	
	
	
	
	
}
