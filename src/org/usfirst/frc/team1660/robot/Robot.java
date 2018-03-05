/*
 * Code for the Harlem Knights (FRC 1660) Robot for 2018
 * website: www.hk1660.com
 * online repository: www.github.com/hk1660/frc2018
 */

package org.usfirst.frc.team1660.robot;


/*-----IMPORTED LIBRARIES-----*/
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.CameraServer;


public class Robot<m_robotDrive> extends IterativeRobot {

	Joystick driverStick = new Joystick(RobotMap.DRIVER_JOYSTICK_PORT);
	Joystick maniStick = new Joystick(RobotMap.MANIPULATOR_JOYSTICK_PORT);
	HKDrive hkdrive = new HKDrive(driverStick);
	Lift liftMani = new Lift(maniStick);
	Mouth mouthMani = new Mouth(maniStick);
	//Lidar3 laser3 = new Lidar3();
	@SuppressWarnings("rawtypes")
	SendableChooser strategy = new SendableChooser();
	@SuppressWarnings("rawtypes")
	SendableChooser position = new SendableChooser();
	Timer timerAuto = new Timer();
	AirPressureSensor pressureSensor = new AirPressureSensor(RobotMap.PRESSURE_SENSOR_PORT);

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
		//laser3.startMeasuring();
		
		CameraServer.getInstance().startAutomaticCapture();
		LedStrip.ledInit();
		//updateLidarDistance();
		
		// Auto mode strategies setup
		strategy.addDefault("justCrossAutoLineStrategy(1)", new Integer(1));
		strategy.addObject("simpleSwitchStrategy(2)", new Integer(2));
		strategy.addObject("smartSwitchStrategy(3)", new Integer(3));
		strategy.addObject("smartSwitchLidarStrategy(4)", new Integer(4));
		strategy.addObject("simpleScaleStrategy(5)", new Integer(5));
		SmartDashboard.putData("strategy selector", strategy); 

		position.addDefault("Left(1)", new Integer(1));
		position.addObject("Middle(2)", new Integer(2));
		position.addObject("Right(3)", new Integer(3));
		SmartDashboard.putData("position selector", position); 
	}


	//AUTONOMOUS MODE

	/* Autonomous INIT Stuff \o/ -Khalil */
	@SuppressWarnings("unchecked")
	public void autonomousInit() {

		
		currentStrategy = (int) strategy.getSelected();
		currentPosition = (int) position.getSelected();
		timerAuto.start();
		
		hkdrive.setOffsetAngle();

		angleToOurSwitchPlate = getAngleToSwitchPlate();

		liftMani.flipMouth();

		//laser3.startMeasuring();
		//updateLidarDistance();

		newTravelFlag = true;
		this.mouthMani.shutUp();
		
		hkdrive.yawZeroing();

	}

	//AUTO_PERIODIC
	public void autonomousPeriodic() {

		//get current Auto time for SmartDash
		double autoTime = timerAuto.get();
		SmartDashboard.putNumber("autoTime",autoTime);

		//updateLidarDistance();

		//currentStrategy = 3;
		if((currentStrategy== 3) || (currentStrategy ==5)) { //making sure we dont go crossfield
			if ((( currentPosition == 3) &&  (getAngleToSwitchPlate()==90)) || 
			(( currentPosition == 1) &&  (getAngleToSwitchPlate()==-90))){
				//currentStrategy = 1;
			}
		}
		
		SmartDashboard.putNumber("actual strategy", currentStrategy);

		//deciding on which strategy to run
		if(currentStrategy == 1) { 
			this.justCrossAutolineStrategy(autoTime);
		} else if (currentStrategy == 2) {
			this.simpleSwitchStrategy(autoTime);
		} else if (currentStrategy == 3) {
			this.smartSwitchStrategy(autoTime);
		} 
		
		/*else if (currentStrategy == 4) {
			this.smartSwitchLidarStrategy(autoTime);
		} 
		*/
		
		else if (currentStrategy == 5) {
			this.simpleScaleStrategy(autoTime);
		}

	}

	//TELEOP MODE
	public void teleopInit() { 
		//this.updateLidarDistance();
		//laser3.stopMeasuring();
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
		
		pressureSensor.updateAirPressureDisplay();
		//updateLidarDistance();	

	}


	/*----- AUTONOMOUS STRATEGY METHODS -----*/

	//QUICK AUTO STRATEGY #1: this is to get to the auto line assuming position 1 or 3 -Marlahna
	public void justCrossAutolineStrategy (double timeA) { 

		double forwardVoltage = 6.0;

		double startPauseTime = .5;									//0.5	
		double firstForwardTime = 2.75 + startPauseTime;			//3.25

		if(timeA < startPauseTime){
		}else if (timeA < firstForwardTime) {
			hkdrive.goForwardVoltage(forwardVoltage);
			liftMani.elevatorLift(liftMani.switchHeight);	
		}else{
			hkdrive.stop();	
		}
	}


	//AUTO STRATEGY #2: Starting from position 1 or 3, go forward to a particular plate of our switch, drop off the powercube if its the correct one -Marlahna
	public void simpleSwitchStrategy(double timeB) {

		double forwardVoltage = 6.0;
		//double turnVoltage = 8.0;
		double turnAngle = 90.0;

		//position 3 needs a neg turn, position 1 needs a positive turn
		if(currentPosition == 3) {
			//turnVoltage *= -1;
			turnAngle *= -1;
		}

		double startPauseTime = .5;								//1.0	
		double firstForwardTime = 2.25 + startPauseTime;		//2.0
		double firstTurnTime = 3.0 +firstForwardTime;			//2.5
		double secondForwardTime = .5 + firstTurnTime;			//3.5
		double dipTime = 0.6 + secondForwardTime;				//7.0
		double lastTime = 6.0;

		if(timeB < startPauseTime){
		}else if (timeB < firstForwardTime) {
			hkdrive.goForwardVoltage(forwardVoltage);
			liftMani.elevatorLift(liftMani.switchHeight);	
		}else if(timeB < firstTurnTime) {
			//hkdrive.turnVoltage(turnVoltage);
			hkdrive.autoTurn(turnAngle);
		}else if(timeB < secondForwardTime) {
			hkdrive.goForwardVoltage(forwardVoltage);
		}

		else if((currentPosition == 1 && getAngleToSwitchPlate() == -90
				|| currentPosition == 3 && getAngleToSwitchPlate() == 90 )
				&& timeB<lastTime) { 

			if (timeB < dipTime){
				hkdrive.stop();										//stop driving
				liftMani.dipMouth();								//let the mouth "Dip"
			}else if(timeB < lastTime){
				mouthMani.spit();									//spit out powercube
			} 
		}
		else{
			hkdrive.stop();	
			this.mouthMani.shutUp();
			//stop driving
		}


	}

	//AUTO STRATEGY #3: Starting in position 2, move to the correct switch plate and drop off the powercube -Marlahna
	//Rickeya,Jocelyn,Mohamed C 
	public void smartSwitchStrategy(double timeC) {

		double forwardVoltage = 6.0;
		double turnVoltage = 8.0;

		//check which direction to go based on plate color
		if(this.getAngleToSwitchPlate() == -90) {
			turnVoltage *= -1;
		}

		double startPauseTime = 0.5;							//0.5	
		double firstForwardTime = 0.5 + startPauseTime;			//1.0
		double firstTurnTime = 3.0 +firstForwardTime;			//1.625
		double secondForwardTime = 1.1250 + firstTurnTime;		//2.75
		double secondTurnTime = 3.0 + secondForwardTime;		//3.375
		double forwardToSwitchTime = 1.0 + secondTurnTime;		//5.575
		double dipTime = 0.3 + forwardToSwitchTime;				//5.875
		double lastTime = 8.0;

		if(timeC < startPauseTime){
		}else if (timeC < firstForwardTime) {
			//hkdrive.goForwardFacing(forwardVoltage, 0.0);
				SmartDashboard.putString("auto move", "Going forward");
			hkdrive.goForwardVoltage(forwardVoltage);
		}else if(timeC < firstTurnTime) {
			hkdrive.autoTurn(angleToOurSwitchPlate);
				SmartDashboard.putString("auto move", "turning");
			//hkdrive.turnVoltage(turnVoltage);
		}else if(timeC < secondForwardTime) {
			//hkdrive.goForwardFacing(forwardVoltage, angleToOurSwitchPlate);
				SmartDashboard.putString("auto move", "Going forward");
			hkdrive.goForwardVoltage(forwardVoltage);
		}else if(timeC < secondTurnTime) {
			hkdrive.autoTurn(0.0);
				SmartDashboard.putString("auto move", "turning");
			//hkdrive.turnVoltage(-turnVoltage);
		}else if(timeC < forwardToSwitchTime) {
			//hkdrive.goForwardFacing(forwardVoltage, -angleToOurSwitchPlate);
			hkdrive.goForwardVoltage(forwardVoltage);
			liftMani.elevatorLift(liftMani.switchHeight);		//bring the cube "up"
		}else if (timeC < dipTime){
			hkdrive.stop();										//stop driving
			liftMani.dipMouth();								//let the mouth "Dip"
		}else if(timeC < lastTime){
			mouthMani.spit();									//spit out powercube
		}else{
			hkdrive.stop();										//stop driving
		}

	}

	
	
/*	
	//AUTO STRATEGY #4: Starting in position 2, move to the correct switch plate and drop off the powercube -Marlahna
	//Khalil & Mohamed updated with Shubham
	public void smartSwitchLidarStrategy(double timeD) {

		SmartDashboard.putNumber("autoTime", timeD);
		double forwardVoltage = 6.0;

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
			hkdrive.goForwardVoltage(forwardVoltage);
		} 
		
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
		 
		else{
			hkdrive.stop();										//stop driving
		}

	}

*/
	
	//AUTO STRATEGY #5: Go forward to the scale and drop off a cube if its the correct plate
	public void simpleScaleStrategy(double timeE) {
		
		double forwardVoltage = 5.5;
		double turnAngle = 90.0;

		//position 3 needs a neg turn, position 1 needs a positive turn
		if(currentPosition == 3) {
			turnAngle *= -1;
		}

		double startPauseTime = 0;								
		double firstForwardTime = 6.5 + startPauseTime;		//6
		double firstTurnTime = 2.0 +firstForwardTime;			//8
		double secondForwardTime = 0.7 + firstTurnTime;			//8.5
		double spitTime = 14.5;
		double lastTime = 15;

		if(timeE < startPauseTime){
			
		}else if (timeE < firstForwardTime) {
			hkdrive.goForwardVoltage(forwardVoltage);
			liftMani.elevatorLift(liftMani.topHeight);	
		}else if(timeE < firstTurnTime) {
			hkdrive.autoTurn(turnAngle);
		}else if(timeE < secondForwardTime) {
			hkdrive.goForwardVoltage(forwardVoltage); 
			
		} else if (timeE < spitTime) {
			hkdrive.stop();
			mouthMani.spit();									//spit out powercube
		}
		else if(timeE < lastTime){
			//hkdrive.goForwardVoltage(forwardVoltage); 
			mouthMani.spit();									//spit out powercube
		} 
		else{
		hkdrive.stop();	
		this.mouthMani.shutUp();

		}
	}
	//AUTO STRATEGY #6 POSITION 2 SCALE
	public void smartScaleStrategy (double timeR) {
		double forwardVoltage = 5.5;
		double turnAngle = 90.0;
		//check which direction to go based on plate color
		//check which direction to go based on plate color
				if(this.getAngleToSwitchPlate() == -90) {
					turnAngle *= -1;
		
				

				double startPauseTime = 0.5;							//0.5	
				double firstForwardTime = 0.5 + startPauseTime;			//1.0
				double firstTurnTime = 3.0 +firstForwardTime;			//1.625
				double secondForwardTime = 3.1250 + firstTurnTime;		//2.75
				double secondTurnTime = 3.0 + secondForwardTime;		//3.375
				double forwardToScaleTime = 3.0 + secondTurnTime;		//5.575
				double dipTime = 0.3 + forwardToScaleTime;				//5.875
				double lastTime = 8.0;

				if(timeR < startPauseTime){
				}else if (timeR < firstForwardTime) {
					//hkdrive.goForwardFacing(forwardVoltage, 0.0);
					hkdrive.goForwardVoltage(forwardVoltage);
				}else if(timeR < firstTurnTime) {
					hkdrive.autoTurn(angleToOurSwitchPlate);
					//hkdrive.turnVoltage(turnVoltage);
				}else if(timeR < secondForwardTime) {
					//hkdrive.goForwardFacing(forwardVoltage, angleToOurSwitchPlate);
					hkdrive.goForwardVoltage(forwardVoltage);
				}else if(timeR < secondTurnTime) {
					hkdrive.autoTurn(0.0);
					//hkdrive.turnVoltage(-turnVoltage);
				}else if(timeR < forwardToScaleTime) {
					//hkdrive.goForwardFacing(forwardVoltage, -angleToOurSwitchPlate);
					hkdrive.goForwardVoltage(forwardVoltage);
					liftMani.elevatorLift(liftMani.switchHeight);		//bring the cube "up"
				}else if (timeR < dipTime){
					hkdrive.stop();										//stop driving
					liftMani.dipMouth();								//let the mouth "Dip"
				}else if(timeR < lastTime){
					mouthMani.spit();									//spit out powercube
				}else{
					hkdrive.stop();										//stop driving
				}

				}
	}


	public void updateLidarDistance(){
		//lidarDistance = laser3.pidGet();
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

