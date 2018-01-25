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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class Robot<m_robotDrive> extends IterativeRobot {

	//JOYSTICK - Nana B. & Mathew W.
	Joystick driverStick = new Joystick(0);
	Joystick manipStick = new Joystick(1);
	HKDrive hkdrive = new HKDrive(driverStick);
	Lift liftMani = new Lift(manipStick);
	Mouth mouthMani = new Mouth(manipStick);
	Lidar laser = new Lidar();
	
	
	/*----- REQUIRED FRC MAIN METHODS -----*/
	public void robotInit() {

		hkdrive.driveInit();		//intialize the HKDrive speed controllers
		liftMani.liftInit();
		mouthMani.mouthInit();

		laser.initLidar();
	}

	//AUTONOMOUS MODE
	public void autonomousInit() {
		//autocode that is used for every strategy goes here @AmadouGamby & @marlahna
		//Create an auto timer here
		// timer.reset(); // Resets the timer to 0
		//timer.start(); // Start counting
		hkdrive.resetAngle();
	}
	
	public void autonomousPeriodic() {

	}

	//TELEOP MODE
	public void teleopInit() { 
		//hkdrive.resetAngle();  //delete later
	}
	
	public void teleopPeriodic() {

		hkdrive.checkDriving();
		hkdrive.checkAutoTurn();
		hkdrive.getCurrentAngle();
		hkdrive.checkResetAngle();

		liftMani.getEncoder();
		liftMani.elevatorLift(10);

		laser.getDistance();
		

	}


	
	/*------------------------- CUSTOM METHODS -------------------------*/

	/*----- JOYSTICK METHODS -----*/
	
		

		

	/*----- SENSOR METHODS -----*/

	/*----- BASIC ROBOT MOTION METHODS -----*/


	/*----- AUTONOMOUS STRATEGY METHODS -----*/

}
