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
	//Lidar3 laser3 = new Lidar3();
	SendableChooser strategy = new SendableChooser();
	SendableChooser position = new SendableChooser();
	Timer timerAuto = new Timer();

	/*----- REQUIRED FRC MAIN METHODS -----*/
	public void robotInit() {

		hkdrive.driveInit();		//Initialize the HKDrive speed controllers
		liftMani.liftInit();
		mouthMani.mouthInit();
		//laser.initLidar();
		//laser3.startMeasuring();

	}

	
	//AUTONOMOUS MODE

	/* Autonomous Stuff \o/ -Khalil */
	public void autonomousInit() {

		//AUTO_INIT

		// Auto mode strategies setup
		strategy.addDefault("goToRowOne", new Integer(1));
		strategy.addObject("JustCrossAutoline", new Integer(2));
		strategy.addObject("runToSwitchDecideDirectionDrop", new Integer(3));
		strategy.addObject("runToScaleDrop", new Integer(4));
		SmartDashboard.putData("strategy selector", strategy); 

		position.addDefault("Left", new Integer(1));
		position.addObject("Middle", new Integer(2));
		position.addObject("Right", new Integer(3));
		SmartDashboard.putData("position selector", position); 

		
		
	}
	
		//AUTO_PERIODIC
	public void autonomousPeriodic() {
		
		//gets the direction of our alliance plates from FMS (OurSwitch > Scale > OtherSwitch)
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		char rowOne;
		char rowTwo;
		char rowThree;
		int angle;

	
		timerAuto.start();
		int currentStrategy = (int) strategy.getSelected();
		int currentPosition = (int) position.getSelected();
		hkdrive.setOffsetAngle();


		//stores the plate colors
		rowOne = gameData.charAt(0);
		rowTwo = gameData.charAt(1);
		rowThree = gameData.charAt(2);
		if(rowOne == 'L') {
			angle = -90;
		}else angle = 90;
	

		while(isAutonomous() && isEnabled()){ 

			//get current Auto time for SmartDash
			double autoTime = timerAuto.get();
			SmartDashboard.putNumber("autoTime",autoTime);


			//deciding on which strategy to run
			if(currentStrategy == 1) { //based off time movement 
				if(currentPosition == 1 && rowOne== 'L') {
				 goToRowOne(autoTime, currentPosition, angle);}
				 else if(currentPosition ==3 && rowOne =='R') {
					 goToRowOne(autoTime, currentPosition, angle);
				 }else if(currentPosition == 2) {
					 movingFromPositionTwo(autoTime, angle);}
				 else {
					 JustCrossAutoline(autoTime); //position 1 and r or position 3 and L
					 
					  }
			} else if (currentStrategy == 2){ 
				JustCrossAutoline(autoTime); 
			} else if (currentStrategy == 3){
				//runToSwitchDecideDirectionDrop(laser3, currentPosition, rowTwo);
			} else if (currentStrategy == 4){
				//runToScaleDrop(laser3, currentPosition, rowTwo);
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
		hkdrive.checkSetOffsetAngle();

		mouthMani.checkEatSpit();

		liftMani.getEncoder();
		liftMani.checkEncoderZero();
		liftMani.checkLiftPoints();
		liftMani.checkElevatorLift();
		
		liftMani.checkClimb();
		liftMani.checkDip();
		liftMani.checkFlip();
		

		//laser.getDistance();
		//laser3.getDistance();
		SmartDashboard.putNumber("lidar value", laser2.getDistance_inches());

	}


	/* public void something(){
		
	}
*/
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
		//rendezvousPointAutoline(timeB);
		if(timeB < 8.0){
			hkdrive.goForwardPercentOutput(0.5);
			hkdrive.autoTurn(90.0);}
		else { hkdrive.stop();
			mouthMani.spit();
		}

		}
	

	

	//AUTO STRATEGY #4: Go forward to the scale and drop off a cube if its the correct plate
	public void runToScaleDrop(double timeD, int position, char row) {


	}

	//QUICK AUTO Methods to get to certain points
	public void JustCrossAutoline (double timeF) { //this is to get to the auto line assuming position 1 and 3
		if(timeF < 2.75){
			hkdrive.goForwardPercentOutput(0.5);
	}
		else {
			hkdrive.stop();
		}
	}
	public void movingFromPositionTwo (double timeG, int angleDegree) { //assuming in position 2 
		if (timeG < 1.0) {
			hkdrive.goForwardPercentOutput(.5);
		} else if(timeG <1.5) {
			hkdrive.autoTurn(angleDegree);		
		}else if(timeG < 2.5) {
			hkdrive.goForwardPercentOutput(.5);
		}else if(timeG < 3.0) {
			hkdrive.autoTurn(-angleDegree);
		}else if(timeG < 5.0) {
			hkdrive.goForwardPercentOutput(.5);
		}else if(timeG <5.5) {
			hkdrive.autoTurn(angleDegree);
		}else if (timeG < 7)
			hkdrive.goForwardPercentOutput(.5);
		else {
			hkdrive.stop();
			mouthMani.spit();
		}
		
	}

	
	public void goToRowOne (double timeH, int position, int angleDegree) {
		if(timeH < 2.5) {
			hkdrive.goForwardPercentOutput(.5);
		}
		else if(timeH < 2.9) {
			hkdrive.autoTurn(angleDegree);
		}else if(timeH < 4.0) {
			hkdrive.goForwardPercentOutput(.5);
		}
		else if(timeH < 4.3) {
		hkdrive.autoTurn(-angleDegree);}
		
		else if(timeH < 5.5) {
			hkdrive.goForwardPercentOutput(.5);
		}
		else if (timeH < 5.8) {
			hkdrive.autoTurn(-angleDegree);
		}
		else if(timeH < 6.2) {
			hkdrive.goForwardPercentOutput(.5);
		}
			else {
			mouthMani.spit();
	}
	}
	

}
	
	
	