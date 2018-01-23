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

	
	
	//Manipulator Declarations
	private static final int kMouthMotorChannel = 4;
	private static final int kLiftMotorChannel = 5;	
	private static final int kSwitchBoi = 6; 			//lol we can switch the name l8r

	private WPI_TalonSRX liftMotor;
	private WPI_TalonSRX mouthMotor;

	//JOYSTICK - Nana B. & Mathew W.
	Joystick driverStick = new Joystick(0);
	Joystick manipStick = new Joystick(1);
	final static int A_BUTTON = 1;
	final static int B_BUTTON = 2;
	final static int X_BUTTON = 3;
	final static int Y_BUTTON = 4;
	final static int LB_BUTTON = 5;
	final static int RB_BUTTON = 6;
	final static int BACK_BUTTON = 7;
	final static int START_BUTTON = 8;
	final static int LEFT_JOY_BUTTON = 9;
	final static int RIGHT_JOY_BUTTON = 10;
	final static int LEFT_X_AXIS = 0;
	final static int LEFT_Y_AXIS = 1;
	final static int LT_AXIS = 2;
	final static int RT_AXIS = 3;
	final static int RIGHT_X_AXIS = 4;
	final static int RIGHT_Y_AXIS = 5;
	final static int POV_UP = 0;
	final static int POV_LEFT = 270;
	final static int POV_DOWN = 180;
	final static int POV_RIGHT = 90;


	HKDrive hkdrive = new HKDrive(driverStick);
	
	/*----- REQUIRED FRC MAIN METHODS -----*/
	public void robotInit() {

		//Manipulator Initializations
		mouthMotor = new WPI_TalonSRX(kMouthMotorChannel);
		liftMotor = new WPI_TalonSRX(kLiftMotorChannel); //A.K.A Elevator/Climb Manipulator


	}

	//AUTONOMOUS MODE
/*	public void autonomousInit() {
		//autocode that is used for every strategy goes here @AmadouGamby & @marlahna
		// timer.reset(); // Resets the timer to 0
		//timer.start(); // Start counting
		hkdrive.resetAngle();
	}*/	
	public void autonomousPeriodic() {

	}

	//TELEOP MODE
	public void teleopInit() { 
		hkdrive.resetAngle();  //delete later
	}
	
	public void teleopPeriodic() {

		hkdrive.checkDriving();
	//	hkdrive.checkResetAngle();

		
	//	hkdrive.getCurrentAngle();
		this.getEncoder();

	}


	
	/*------------------------- CUSTOM METHODS -------------------------*/

	/*----- JOYSTICK METHODS -----*/
	
		

		

	/*----- SENSOR METHODS -----*/

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
