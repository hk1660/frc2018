/*
 * Code for the Harlem Knights (FRC 1660) Robot for 2018
 * website: www.hk1660.com
 * online repository: www.github.com/hk1660/frc2018
 */

package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;

/*-----IMPORTED LIBRARIES-----*/

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;


public class Robot<m_robotDrive> extends IterativeRobot {

	Joystick driverStick = new Joystick(0);
	Joystick maniStick = new Joystick(1);
	HKDrive hkdrive = new HKDrive(driverStick);
	Lift liftMani = new Lift(maniStick);
	Mouth mouthMani = new Mouth(maniStick);
	Lidar laser = new Lidar();
	SendableChooser strategy = new SendableChooser();
	SendableChooser position = new SendableChooser();
	Timer timerAuto = new Timer();

	/*----- REQUIRED FRC MAIN METHODS -----*/
	public void robotInit() {

		hkdrive.driveInit();		//Initialize the HKDrive speed controllers
		liftMani.liftInit();
		mouthMani.mouthInit();
		laser.initLidar();


		/* Auto mode strategies */
		strategy.addDefault("move Forward", new Integer(1));
		strategy.addObject("move to Switch", new Integer(2));
		strategy.addObject("move to Scale", new Integer(3));
		SmartDashboard.putData("strategy selector", strategy); 

		position.addDefault("Left", new Integer(1));
		position.addObject("Right", new Integer(2));
		position.addObject("Center", new Integer(3));
		SmartDashboard.putData("position selector", position); 
	}

	//AUTONOMOUS MODE

	/* Autonomous Stuff \o/ -Khalil */
	public void autonomous() {

		//AUTO_INIT

		//gets the direction of our alliance plates from FMS (OurSwitch > Scale > OtherSwitch)
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		char rowOne;
		char rowTwo;
		char rowThree;

		timerAuto.start();
		int currentStrategy = (int) strategy.getSelected();
		int currentPosition = (int) position.getSelected();
		hkdrive.resetAngle();

		//AUTO_PERIODIC
		while(isAutonomous() && isEnabled()){ 

			//get current Auto time for SmartDash
			double autoTime = timerAuto.get();
			SmartDashboard.putNumber("autoTime",autoTime);

			//stores the plate colors
			rowOne = gameData.charAt(0);
			rowTwo = gameData.charAt(1);
			rowThree = gameData.charAt(2);

			/* 			Strategies need to be made before this is continued further

			 */

			if(currentStrategy == 1) {
				runGoForwardOnly(autoTime, currentPosition);
			} else if (currentStrategy == 2){
				runToSwitchSimpleDrop(autoTime, currentPosition, rowOne);
			} else if (currentStrategy == 3){
				runToSwitchDecideDirectionDrop(autoTime, currentPosition, rowTwo);
			} else if (currentStrategy == 4){
				runToScaleDrop(autoTime, currentPosition, rowTwo);
			}

		}

	}
	
	public void runGoForwardOnly(double timeA, int position ) {

		//go forward for times 0.0 - 2.0
		if(timeA < 2.0){
			hkdrive.goForwardPercentOutput(0.5);
		}
		
		//stop
		else {
			hkdrive.stop();
		}


	}

	public void runToSwitchSimpleDrop(double timeB, int position, char row) {

		//go forward for times 0.0 - 2.0
		if(timeB < 2.0){
			hkdrive.goForwardPercentOutput(0.5);
		}
		
		//spit out powercube
		else if(timeB > 2.0 && timeB < 3.0){
			hkdrive.goForwardPercentOutput(0.5);
		}

		//stop
		else {
			hkdrive.stop();
		}

	}

	public void runToSwitchDecideDirectionDrop(double timeC, int position, char row) {


	}

	public void runToScaleDrop(double timeD, int position, char row) {


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

		mouthMani.checkEatSpit();

		liftMani.getEncoder();
		liftMani.checkEncoderZero();
		liftMani.checkLiftPoints();
		liftMani.checkElevatorLift();

		laser.getDistance();


	}



	/*------------------------- CUSTOM METHODS -------------------------*/

	/*----- AUTONOMOUS STRATEGY METHODS -----*/


}


