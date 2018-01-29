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

	Joystick driverStick = new Joystick(RobotMap.DRIVER_JOYSTICK_PORT);
	Joystick maniStick = new Joystick(RobotMap.MANIPULATOR_JOYSTICK_PORT);
	HKDrive hkdrive = new HKDrive(driverStick);
	Lift liftMani = new Lift(maniStick);
	Mouth mouthMani = new Mouth(maniStick);
	//Lidar laser = new Lidar();
	Lidar2 laser2 = new Lidar2();
	SendableChooser strategy = new SendableChooser();
	SendableChooser position = new SendableChooser();
	Timer timerAuto = new Timer();

	/*----- REQUIRED FRC MAIN METHODS -----*/
	public void robotInit() {

		hkdrive.driveInit();		//Initialize the HKDrive speed controllers
		liftMani.liftInit();
		mouthMani.mouthInit();
		//laser.initLidar();

	}

	
	//AUTONOMOUS MODE

	/* Autonomous Stuff \o/ -Khalil */
	public void autonomous() {

		//AUTO_INIT

		// Auto mode strategies setup
		strategy.addDefault("runGoForwardOnly", new Integer(1));
		strategy.addObject("runToSwitchSimpleDrop", new Integer(2));
		strategy.addObject("runToSwitchDecideDirectionDrop", new Integer(3));
		strategy.addObject("runToScaleDrop", new Integer(4));
		SmartDashboard.putData("strategy selector", strategy); 

		position.addDefault("Left", new Integer(1));
		position.addObject("Middle", new Integer(2));
		position.addObject("Right", new Integer(3));
		SmartDashboard.putData("position selector", position); 

		
		//gets the direction of our alliance plates from FMS (OurSwitch > Scale > OtherSwitch)
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		char rowOne;
		char rowTwo;
		char rowThree;

		timerAuto.start();
		int currentStrategy = (int) strategy.getSelected();
		int currentPosition = (int) position.getSelected();
		hkdrive.setOffsetAngle();


		//stores the plate colors
		rowOne = gameData.charAt(0);
		rowTwo = gameData.charAt(1);
		rowThree = gameData.charAt(2);
		
		//AUTO_PERIODIC
		while(isAutonomous() && isEnabled()){ 

			//get current Auto time for SmartDash
			double autoTime = timerAuto.get();
			SmartDashboard.putNumber("autoTime",autoTime);


			//deciding on which strategy to run
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


	//TELEOP MODE
	public void teleopInit() { 

	}

	public void teleopPeriodic() {

		hkdrive.checkDriving();
		hkdrive.checkAutoTurn();
		hkdrive.getCurrentAngle();
		hkdrive.setOffsetAngle();

		mouthMani.checkEatSpit();

		liftMani.getEncoder();
		liftMani.checkEncoderZero();
		liftMani.checkLiftPoints();
		liftMani.checkElevatorLift();

		//laser.getDistance();
		laser2.getDistance_inches();

	}




	/*----- AUTONOMOUS STRATEGY METHODS -----*/
	
	//AUTO STRATEGY #1: Go forward for 2 seconds and cross the autoline -Marlahna
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

	//AUTO STRATEGY #2: Go forward to a particular plate of our switch, drop off the powercube if its the correct one -Marlahna
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

	//AUTO STRATEGY #3: Based on the correct location of our alliance's switch plate, travel in that direction and drop off a powercube
	public void runToSwitchDecideDirectionDrop(double timeC, int position, char row) {

	}

	//AUTO STRATEGY #4: Go forward to the scale and drop off a cube if its the correct plate
	public void runToScaleDrop(double timeD, int position, char row) {


	}






}


