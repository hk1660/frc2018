/*
 * Code for the Harlem Knights (FRC 1660) Robot for 2018
 * website: www.hk1660.com
 * online repository: www.github.com/hk1660/frc2018
 */

package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CameraServer;

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
	
	double angleToOurSwitchPlate;
	int currentStrategy;
	int currentPosition;
	
	private double lidarDistance;
	private double startValueDistance = 0.0;
	private double travelDistance = 0.0;
	private boolean newTravelFlag;

	/*----- REQUIRED FRC MAIN METHODS -----*/
	public void robotInit() {

		hkdrive.driveInit();		//Initialize the HKDrive speed controllers
		liftMani.liftInit();
		mouthMani.mouthInit();
		laser3.startMeasuring();

		CameraServer.getInstance().startAutomaticCapture();
		LedStrip.ledInit();
		updateLidarDistance();
		
	}


	//AUTONOMOUS MODE

	/* Autonomous INIT Stuff \o/ -Khalil */
	@SuppressWarnings("unchecked")
	public void autonomousInit() {

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

		timerAuto.start();
		currentStrategy = (int) strategy.getSelected();
		currentPosition = (int) position.getSelected();
		hkdrive.setOffsetAngle();

		angleToOurSwitchPlate = getAngleToSwitchPlate();

		liftMani.flipMouth();

		laser3.startMeasuring();
		updateLidarDistance();
		
		newTravelFlag = true;

		//  hkdrive.setSafetyEnabled(false);

	}

	//AUTO_PERIODIC
	public void autonomousPeriodic() {

		//while(isAutonomous() && isEnabled()){ 

		//get current Auto time for SmartDash
		double autoTime = timerAuto.get();
		SmartDashboard.putNumber("autoTime",autoTime);

		updateLidarDistance();

		currentStrategy = 4;

		//deciding on which strategy to run
		if(currentStrategy == 1) { 
			this.justCrossAutolineStrategy(autoTime);
		} else if (currentStrategy == 2) {
			this.simpleSwitchStrategy(autoTime, currentPosition);
		} else if (currentStrategy == 3) {
			this.smartSwitchStrategy(autoTime);
		} else if (currentStrategy == 4) {
			this.smartSwitchLidarStrategy(autoTime);
		} else if (currentStrategy == 5) {
			this.simpleScaleStrategy(autoTime, currentPosition);
		}

	}

	//TELEOP MODE
	public void teleopInit() { 
		this.updateLidarDistance();
		laser3.stopMeasuring();
	}

	public void teleopPeriodic() {
		
		this.checkLed();

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
		liftMani.checkFlipDip();
		liftMani.checkLockUnlock();
		liftMani.checkDisengageMotor();

		updateLidarDistance();	

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
	public void simpleSwitchStrategy(double timeB, int position) {

	}

	//AUTO STRATEGY #3: Staring in position 2, move to the correct switch plate and drop off the powercube -Marlahna
	//Rickeya,Jocelyn,Mohamed C 
	public void smartSwitchStrategy(double timeC) {

		//double forwardSpeed = 0.5;
		double forwardVoltage = 6.0;
		double turnVoltage = 6.0;

		double startPauseTime = 1.0;						//1.0	
		double firstForwardTime = 1.0 + startPauseTime;			//2.0
		double firstTurnTime = 0.5 + firstForwardTime;			//2.5
		double secondForwardTime = 1.0 + firstTurnTime;			//3.5
		double secondTurnTime = 0.5 + secondForwardTime;		//4.0
		double forwardToSwitchTime = 2.0 + secondTurnTime;		//6.0
		double dipTime = 1.0 + forwardToSwitchTime;				//7.0
		double endAutoTime = 10.0;

		if(timeC < startPauseTime){
			liftMani.flipMouth();								//start with mouth "Flip"
		}else if (timeC < firstForwardTime) {
			hkdrive.goForwardVoltage(forwardVoltage);
			//hkdrive.goForwardPercentOutput(forwardSpeed);
		}else if(timeC < firstTurnTime) {
			hkdrive.turnVoltage(turnVoltage);
			//hkdrive.autoTurn(angleToOurSwitchPlate);		
		}else if(timeC < secondForwardTime) {
			hkdrive.goForwardVoltage(forwardVoltage);
			//hkdrive.goForwardPercentOutput(forwardSpeed);
		}else if(timeC < secondTurnTime) {
			hkdrive.turnVoltage(-turnVoltage);
			//hkdrive.autoTurn(-angleToOurSwitchPlate);
		}else if(timeC < forwardToSwitchTime) {
			hkdrive.goForwardVoltage(forwardVoltage);
			//hkdrive.goForwardPercentOutput(forwardSpeed);
			liftMani.elevatorLift(liftMani.switchHeight);		//bring the cube "up"
		}else if (timeC < dipTime){
			hkdrive.stop();										//stop driving
			liftMani.dipMouth();								//let the mouth "Dip"
		}else if(timeC < endAutoTime){
			mouthMani.spit();									//spit out powercube
		}else{
			hkdrive.stop();										//stop driving
		}

	}

	//AUTO STRATEGY #4: Starting in position 2, move to the correct switch plate and drop off the powercube -Marlahna
	//Khalil & Mohamed updated with Shubham
	public void smartSwitchLidarStrategy(double timeD) {

		SmartDashboard.putNumber("autoTime", timeD);
		double forwardSpeed = 0.4;

		double startPauseTime = 1.0;							//1.0	
		double firstForwardTime = 5.0 + startPauseTime;			//2.0
		double firstTurnTime = 0.5 + firstForwardTime;			//2.5
		double secondForwardTime = 1.0 + firstTurnTime;			//3.5
		double secondTurnTime = 0.5 + secondForwardTime;		//4.0
		double forwardToSwitchTime = 2.0 + secondTurnTime;		//6.0
		double dipTime = 1.0 + forwardToSwitchTime;				//7.0
		double endAutoTime = 10.0;

		double dist1inches = 30.00;		//inches to go fwd off wall  30
		double dist2inches = 48.0;		//inches to go left or right
		double dist3inches = 73.0;		//inches to go to wall

		updateLidarDistance();

		if(timeD < startPauseTime){
		} else if(timeD < firstForwardTime && hasNotTraveledLidar(dist1inches)){
			hkdrive.goForwardPercentOutput(forwardSpeed);
		} 
		/*
		else if(timeD < firstTurnTime) {
			hkdrive.autoTurn(angleToOurSwitchPlate);	
			resetTargetDistance();
		}

		else if(timeD < secondForwardTime && hasNotTraveled(dist2inches)) {
			hkdrive.goForwardPercentOutput(forwardSpeed);
		}else if(timeD < secondTurnTime) {
			hkdrive.autoTurn(-angleToOurSwitchPlate);
			resetTargetDistance();
		}else if(timeD < forwardToSwitchTime && hasNotReachedTarget(0.0)) {
			hkdrive.goForwardPercentOutput(forwardSpeed);
			liftMani.elevatorLift(liftMani.switchHeight);		//bring the cube "up"
		}else if (timeD < dipTime){
			hkdrive.stop();										//stop driving
			liftMani.dipMouth();								//let the mouth "Dip"
		}else if(timeD < endAutoTime){
			mouthMani.spit();									//spit out powercube
		}
		 */
		else{
			hkdrive.stop();										//stop driving
		}

	}

	//AUTO STRATEGY #5: Go forward to the scale and drop off a cube if its the correct plate
	public void simpleScaleStrategy(double timeE, int position) {


	}


	public void updateLidarDistance(){
		lidarDistance = laser3.pidGet();
		SmartDashboard.putNumber("lidarDistance", lidarDistance);
	}


	public boolean hasNotTraveledLidar(double targetDistance){

		if(newTravelFlag == true){
			startValueDistance = lidarDistance;
			newTravelFlag = false;
		}

		updateLidarDistance();

		travelDistance = startValueDistance - lidarDistance;
		
		SmartDashboard.putNumber("startDistance", startValueDistance);	
		SmartDashboard.putNumber("travelDistance", travelDistance);	

		return (travelDistance < targetDistance);
	}

	public void resetTargetDistance(){
		travelDistance = 0.0;
		newTravelFlag = true;
	}

	public boolean hasNotReachedTargetLidar(double targetDistance){

		double distanceToTarget = lidarDistance - targetDistance;
		SmartDashboard.putNumber("distanceToTarget", distanceToTarget);	
		return distanceToTarget > 0;
	}


	//method that gets the direction of our alliance plates from FMS (OurSwitch > Scale > OtherSwitch)
	public double getAngleToSwitchPlate(){

		String gameData = DriverStation.getInstance().getGameSpecificMessage();

		char ourSwitchPlateSide = gameData.charAt(0);
		char scalePlateSide = gameData.charAt(1);
		char otherSwitchPlateSide = gameData.charAt(2);

		if(ourSwitchPlateSide == 'L') {
			return -90.0;
		}else {
			return 90.0;
		}
	}

	public void checkLed(){

		//double matchTime = DriverStation.getInstance().getMatchTime();
		int thresh = 1000;
		LedStrip.ledAllianceColor();

		/*
		if(matchTime < 30.0 && matchTime > 25.0){
			LedStrip.partyTime();			
		} else if (matchTime < 25.0 
				&& liftMani.getEncoder() < (liftMani.pullUpHeight + thresh) 
				&& liftMani.getEncoder() > (liftMani.pullUpHeight - thresh)){

			LedStrip.ledLockTheClimbNow();
		} else if (matchTime < 5.0 && matchTime > 0.0){
			LedStrip.partyTime();
		} else {
			LedStrip.ledAllianceColor();			
		}

		 */

	}
}

