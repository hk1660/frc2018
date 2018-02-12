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
	//Lidar2 laser2 = new Lidar2();
	Lidar3 laser3 = new Lidar3();
	@SuppressWarnings("rawtypes")
	SendableChooser strategy = new SendableChooser();
	@SuppressWarnings("rawtypes")
	SendableChooser position = new SendableChooser();
	Timer timerAuto = new Timer();
	private double distance;

	/*----- REQUIRED FRC MAIN METHODS -----*/
	public void robotInit() {

		hkdrive.driveInit();		//Initialize the HKDrive speed controllers
		liftMani.liftInit();
		mouthMani.mouthInit();
		//laser.initLidar();
		laser3.startMeasuring();
	}


	//AUTONOMOUS MODE

	/* Autonomous Stuff \o/ -Khalil */
	@SuppressWarnings("unchecked")
	public void autonomousInit() {

		//AUTO_INIT

		// Auto mode strategies setup
		strategy.addDefault("justCrossAutoLineStrategy", new Integer(1));
		strategy.addObject("simpleSwitchStrategy", new Integer(2));
		strategy.addObject("smartSwitchStrategy", new Integer(3));
		strategy.addObject("smartSwitchLidarStrategy", new Integer(4));
		strategy.addObject("simpleScaleStrategy", new Integer(5));
		SmartDashboard.putData("strategy selector", strategy); 

		position.addDefault("Left", new Integer(1));
		position.addObject("Middle", new Integer(2));
		position.addObject("Right", new Integer(3));
		SmartDashboard.putData("position selector", position); 

	}

	//AUTO_PERIODIC
	public void autonomousPeriodic() {

		//distance = laser.getDistance();
		//distance = laser2.getDistance_inches();
		distance = laser3.pidGet();
		SmartDashboard.putNumber("lidar value", distance);

		timerAuto.start();
		int currentStrategy = (int) strategy.getSelected();
		int currentPosition = (int) position.getSelected();
		hkdrive.setOffsetAngle();

		//gets the direction of our alliance plates from FMS (OurSwitch > Scale > OtherSwitch)
		char ourSwitchPlateSide;
		char scalePlateSide;
		char otherSwitchPlateSide;
		int angleToOurSwitchPlate;
		String gameData = DriverStation.getInstance().getGameSpecificMessage();

		ourSwitchPlateSide = gameData.charAt(0);
		scalePlateSide = gameData.charAt(1);
		otherSwitchPlateSide = gameData.charAt(2);
		if(ourSwitchPlateSide == 'L') {
			angleToOurSwitchPlate = -90;
		}else {
			angleToOurSwitchPlate = 90;
		}


		while(isAutonomous() && isEnabled()){ 

			//get current Auto time for SmartDash
			double autoTime = timerAuto.get();
			SmartDashboard.putNumber("autoTime",autoTime);


			//deciding on which strategy to run
			if(currentStrategy == 1) { 
				this.justCrossAutolineStrategy(autoTime);
			} else if (currentStrategy == 2) {
				this.simpleSwitchStrategy(autoTime, angleToOurSwitchPlate, currentPosition);
			} else if (currentStrategy == 3) {
				this.smartSwitchStrategy(autoTime, angleToOurSwitchPlate);
			} else if (currentStrategy == 4) {
				this.smartSwitchLidarStrategy(autoTime, angleToOurSwitchPlate);
			} else if (currentStrategy == 5) {
				this.simpleScaleStrategy(autoTime, angleToOurSwitchPlate, currentPosition);
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
		liftMani.checkManualLift();
		liftMani.checkLiftPoints();
		liftMani.checkClimb();
		liftMani.checkDip();
		liftMani.checkFlip();

		//distance = laser.getDistance();
		//distance = laser2.getDistance_inches();
		distance = laser3.pidGet();
		SmartDashboard.putNumber("lidar value", distance);	

	}


	/*----- AUTONOMOUS STRATEGY METHODS -----*/

	//QUICK AUTO STRATEGY #1: this is to get to the auto line assuming position 1 or 3 -Marlahna
	public void justCrossAutolineStrategy (double timeA) { 
		if(timeA < 2.75){
			hkdrive.goForwardPercentOutput(0.5);
		}
		else {
			hkdrive.stop();
		}
	}


	//AUTO STRATEGY #2: Go forward to a particular plate of our switch, drop off the powercube if its the correct one -Marlahna
	public void simpleSwitchStrategy(double timeB, double angleToPlate, int position) {

	}

	//AUTO STRATEGY #3: Staring in position 2, move to the correct switch plate and drop off the powercube -Marlahna
	public void smartSwitchStrategy(double timeC, double angleToPlate) {

		double forwardSpeed = 0.5;
		double startPause = 1.0;
		double firstForward = 1.0 + startPause;
		double firstTurn = 0.5 + firstForward;
		double secondForward = 1.0 + firstTurn;
		double secondTurn = 0.5 + secondForward;
		double forwardToSwitch = 2.0 + secondTurn;

		if(timeC < startPause){
		
		}else if (timeC < firstForward) {
			hkdrive.goForwardPercentOutput(forwardSpeed);
		}else if(timeC < firstTurn) {
			hkdrive.autoTurn(angleToPlate);		
		}else if(timeC < secondForward) {
			hkdrive.goForwardPercentOutput(forwardSpeed);
		}else if(timeC < secondTurn) {
			hkdrive.autoTurn(-angleToPlate);
		}else if(timeC < forwardToSwitch) {
			hkdrive.goForwardPercentOutput(forwardSpeed);
		}else {
			hkdrive.stop();
			mouthMani.spit();
		}

	}

	//AUTO STRATEGY #4: Starting in position 2, move to the correct switch plate and drop off the powercube -Marlahna
	public void smartSwitchLidarStrategy(double timeD, double angleToPlate) {

		double forwardSpeed = 0.5;
		double startPause = 1.0;
		double firstForward = 1.0 + startPause;
		double firstTurn = 0.5 + firstForward;
		double secondForward = 1.0 + firstTurn;
		double secondTurn = 0.5 + secondForward;
		double forwardToSwitch = 2.0 + secondTurn;

		if (distance < 60.0 && timeD < firstForward) {
			hkdrive.goForwardPercentOutput(forwardSpeed);
		}else if(timeD < firstTurn) {
			hkdrive.autoTurn(angleToPlate);		
		}else if(timeD < secondForward) {
			hkdrive.goForwardPercentOutput(forwardSpeed);
		}else if(timeD < secondTurn) {
			hkdrive.autoTurn(-angleToPlate);
		}else if(timeD < forwardToSwitch) {
			hkdrive.goForwardPercentOutput(forwardSpeed);
		}else {
			hkdrive.stop();
			mouthMani.spit();
		}
	}


	//AUTO STRATEGY #5: Go forward to the scale and drop off a cube if its the correct plate
	public void simpleScaleStrategy(double timeE, double angleToPlate, int position) {


	}




}

