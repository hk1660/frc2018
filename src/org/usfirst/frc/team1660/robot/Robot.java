/*
 * Code for the Harlem Knights (FRC 1660) Robot for 2018
 * website: www.hk1660.com
 * online repository: www.github.com/hk1660/frc2018
 */

package org.usfirst.frc.team1660.robot;

/*-----IMPORTED LIBRARIES-----*/

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;


public class Robot<m_robotDrive> extends IterativeRobot {

	Joystick driverStick = new Joystick(0);
	Joystick maniStick = new Joystick(1);
	HKDrive hkdrive = new HKDrive(driverStick);
	Lift liftMani = new Lift(maniStick);
	Mouth mouthMani = new Mouth(maniStick);
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

		mouthMani.checkEat();
		mouthMani.checkSpit();
		
		liftMani.getEncoder();
		//liftMani.elevatorLift(10);
		liftMani.checkEncoderZero();
		liftMani.checkLiftPoints();
		liftMani.checkElevatorLift();
		
		laser.getDistance();


	}



	/*------------------------- CUSTOM METHODS -------------------------*/

	/*----- JOYSTICK METHODS -----*/


	/*----- SENSOR METHODS -----*/

	/*----- BASIC ROBOT MOTION METHODS -----*/


	/*----- AUTONOMOUS STRATEGY METHODS -----*/

}
