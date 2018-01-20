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
	double zeroedYawPoint = 0.0;

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

	Joystick driverStick = new Joystick(0);
	Joystick manipStick = new Joystick(1);
/*	public void checkDriving()
	{

		double threshold = 0.11;
		double strafe = squareInput(driverStick.getRawAxis(STRAFE_AXIS)) ; // right and left on the left thumb stick?
		double moveValue = squareInput(driverStick.getRawAxis(FORWARDBACKWARD_AXIS));// up and down on left thumb stick?
		double rotateValue = squareInput(driverStick.getRawAxis(TURNSIDEWAYS_AXIS));// right and left on right thumb stick
		double angle = ahrs.getAngle();

		//Kill Ghost motors & turn-off Auto methods if a joystick is pushed	 - Nana B. & Mathew W.
		if(moveValue > -threshold && moveValue < threshold) {
			moveValue = 0.0;
		} */
	private Joystick m_stick;
	
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
		// Use the joystick X axis for lateral movement, Y axis for forward
		// movement, and Z axis for rotation.
		mecDrive.driveCartesian(driverStick.getX(), driverStick.getY(), driverStick.getZ(), 0.0);
		
		getCurrentAngle();
		getEncoder();

	}




	/*----- CUSTOM METHODS -----*/

	// Method to calculate current angle of robot based off of the field -Aldenis
	public int getCurrentAngle(){
		int moddedAngle = Math.floorMod((int)navx.getAngle(), 360);
		int moddedZeroedYawPoint = Math.floorMod((int)zeroedYawPoint, 360);
		int modAngle = Math.floorMod((moddedAngle - moddedZeroedYawPoint), 360);

		SmartDashboard.putNumber("modAngle", modAngle);
		return modAngle;
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
