/*
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

import javax.swing.text.Position;

import edu.wpi.first.wpilibj.CameraServer;


public class Robot<m_robotDrive> extends IterativeRobot {

	Joystick driverStick = new Joystick(RobotMap.DRIVER_JOYSTICK_PORT);
	Joystick maniStick = new Joystick(RobotMap.MANIPULATOR_JOYSTICK_PORT);
	HKDrive hkdrive = new HKDrive(driverStick);
	Lift liftMani = new Lift(maniStick);
	Mouth mouthMani = new Mouth(maniStick);
	//Lidar3 laser3 = new Lidar3();
	
	SendableChooser<Object> strategy = new SendableChooser<Object>();

	SendableChooser<Object> position = new SendableChooser<Object>();
	Timer timerAuto = new Timer();
	

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
		strategy.addObject("smartScaleStrategy(5)", new Integer(5));
		strategy.addObject("Double Switch(6)", new Integer(6));
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

		liftMani.flipMouth();

		//laser3.startMeasuring();
		//updateLidarDistance();

		newTravelFlag = true;
		this.mouthMani.shutUp();

		hkdrive.yawZeroing();

	}

	//AUTO_PERIODIC
	public void autonomousPeriodic() {

		liftMani.limitCompressor();
		
		//get current Auto time for SmartDash
		double autoTime = timerAuto.get();
		SmartDashboard.putNumber("autoTime", autoTime);

		//updateLidarDistance();

		//currentStrategy = 3;	//switch
		//currentPosition = 3;  //right

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
			this.smartScaleStrategy(autoTime);
		}
		else if (currentStrategy == 6) {
			this.doubleSwitch(autoTime);
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
		hkdrive.checkYawZero();

		mouthMani.checkEatSpit();

		liftMani.getEncoder();
		liftMani.checkEncoderZero();
		liftMani.checkManualLift();
		liftMani.checkLiftPoints();
		liftMani.checkClimb();
		liftMani.checkFlipDip();
		liftMani.checkLockUnlock();
		liftMani.checkDisengageMotor();

		liftMani.pressureSensor.updateAirPressureDisplay();
		liftMani.limitCompressor();
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
		double firstForwardTime = 2.308 + startPauseTime;		//2.0
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

		else if(currentPosition == getSwitchPlateSide() && timeB<lastTime) { 

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
	public void smartSwitchStrategy(double timeG) {

		smartSwitchRightAngles(timeG);
		//smartSwitchDiagonal(timeG);
	}

	//AUTO STRATEGY #3A: Using right Angles
	public void smartSwitchRightAngles(double timeC) {
		double forwardVoltage = 6.0;
		double turnAngle = getAngleToSwitchPlate();

		double startPauseTime = 0.01;							//0.5	
		double firstForwardTime = 0.5 + startPauseTime;			//1.0
		double firstTurnTime = 1.0 +firstForwardTime;			//1.625
		double secondForwardTime = 0.35 + firstTurnTime;		//2.75
		double secondTurnTime = 1.0 + secondForwardTime;		//3.375 
		double forwardToSwitchTime = 1.0 + secondTurnTime;		//5.575
		double dipTime = 0.3 + forwardToSwitchTime;				//5.875
		double lastTime = 8.0;

		if(timeC < startPauseTime){
		}else if (timeC < firstForwardTime) {
			//hkdrive.goForwardFacing(forwardVoltage, 0.0);
			liftMani.flipMouth();	SmartDashboard.putString("auto move", "Going forward");
			hkdrive.goForwardVoltage(forwardVoltage);
		}else if(timeC < firstTurnTime) {
			hkdrive.autoTurn(turnAngle);
			SmartDashboard.putString("auto move", "turning");
			//hkdrive.turnVoltage(turnVoltage);
		}else if(timeC < secondForwardTime) {
			liftMani.elevatorLift(liftMani.switchHeight);		//bring the cube "up"
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
			
		}else if (timeC < dipTime){
			hkdrive.stop();										//stop driving
			liftMani.dipMouth();								//let the mouth "Dip"
		}else if(timeC < lastTime){
			mouthMani.spit();									//spit out powercube
		}else{
			hkdrive.stop();										//stop driving
		}

	}

	//AUTO STRATEGY #3B: Using diagonal
	public void smartSwitchDiagonal (double timeH) {
		double driveVoltage = 8.0;
		double diagonalAngle = 45.0;

		//check which direction to go based on plate color
		//if(this.getAngleToSwitchPlate() == -90) {
		if(this.getSwitchPlateSide() == 1) {	//if plate is Left, turn left/negative angle
			diagonalAngle *= -1;
		}

                      		double startPauseTime = 0.01;							//0.1	
		double firstDiagonalTime = 2.8 + startPauseTime;		//2.6
		double forwardToSwitchTime = 0.4 + firstDiagonalTime;	//3.6
		double dipTime = 0.3 + forwardToSwitchTime;				//3.9
		double lastTime = 5.0;

		if(timeH < startPauseTime){
		}else if (timeH < firstDiagonalTime) {
			hkdrive.goDiagonal(driveVoltage, 0.0, diagonalAngle);
			liftMani.elevatorLift(liftMani.switchHeight);		//bring the cube up
			SmartDashboard.putString("auto move", "Diagonal");
		}else if(timeH < forwardToSwitchTime) {

			hkdrive.goStraightAccurate(driveVoltage, 0.0);
			//hkdrive.goForwardVoltage(driveVoltage);
		}else if (timeH < dipTime){
			hkdrive.stop();										//stop driving
			liftMani.dipMouth();								//let the mouth "Dip"
		}else if(timeH < lastTime){
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
	//AUTO STRATEGY #5:
	public void smartScaleStrategy(double timeF) {
		if(getScalePlateSide() == currentPosition) {
			sameScaleStrategy(timeF);
		} else
			snakeScaleStrategy(timeF);

	}

	//AUTO STRATEGY #5a: Go forward to the scale and drop off a cube if its the correct plate
	public void sameScaleStrategy(double timeE) {

		double forwardVoltage = 5.5;
		double turnAngle = 90.0;

		//position 3 needs a neg turn, position 1 needs a positive turn
		if(currentPosition == 3) {
			turnAngle *= -1;

		double startPauseTime = 0;								
		double firstForwardTime = 5.0 + startPauseTime;		//6
		double firstTurnTime = 1.0 +firstForwardTime;			//8
		double secondForwardTime = 0.05 + firstTurnTime;			//8.5
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
	}
	//AUTO STRATEGY #5b POSITION 3/1 SCALE -marlahna Travel to opposite Scal Plate
	public  void snakeScaleStrategy (double timeR) {
		double forwardVoltage = 5.5;
		double turnAngle = 90.0;

		//position 3 needs a neg turn, position 1 needs a positive turn
		if(currentPosition == 3) {
			turnAngle *= -1;
		}

		double startPauseTime = 0;								
		double firstForwardTime = 3.5 + startPauseTime;		//6
		double firstTurnTime = 1.0 +firstForwardTime;			//8
		double secondForwardTime = 2.05 + firstTurnTime; //8.5
		double secondTurnTime = 1.0 + secondForwardTime;
		double thirdForwardTime = .2 + secondTurnTime;
		double spitTime = 14.5 + thirdForwardTime;
		double lastTime = 15;

		if(timeR < startPauseTime){

		}else if (timeR < firstForwardTime) {
			hkdrive.goForwardVoltage(forwardVoltage);
			liftMani.elevatorLift(liftMani.topHeight);	
		}else if(timeR < firstTurnTime) {
			hkdrive.autoTurn(turnAngle);
		}else if(timeR < secondForwardTime) {
			hkdrive.goForwardVoltage(forwardVoltage); 
        } else if (timeR < secondTurnTime) {
        	hkdrive.autoTurn(-turnAngle);
		}else if(timeR < thirdForwardTime) {
			hkdrive.goForwardVoltage(forwardVoltage);
		}else if(timeR < lastTime){
			mouthMani.spit();									//spit out powercube
		} 
		else{
			hkdrive.stop();	
			this.mouthMani.shutUp();
	}
	}
	
	//AUTO STRATEGY #6 DOUBLE CUBE SWITCH FORK
	public void doubleSwitch(double timeF) {
		double forwardVoltage = 6.0;
		double turnAngle = getAngleToSwitchPlate();

		double startPauseTime = 0.01;							//0.5	
		double firstForwardTime = 0.5 + startPauseTime;			//1.0
		double firstTurnTime = 1.0 +firstForwardTime;			//1.625
		double secondForwardTime = 0.35 + firstTurnTime;		//2.75
		double secondTurnTime = 1.0 + secondForwardTime;		//3.375 
		double forwardToSwitchTime = 1.0 + secondTurnTime;		//5.575
		double dipTime = 0.3 + forwardToSwitchTime;				//5.875
		double spitTime = 8.0;									//8.0
		double backFromSwitchTime = 1.0 + spitTime;				//9.0
		double thirdTurnTime = 1.0 + backFromSwitchTime;		//10.0
		double backFromTurnTime = 1.0 + thirdTurnTime;			//11.0
		double fourthTurnTime = 0.5 + backFromTurnTime;			//11.5
		double forwardTimeToCubes = 0.7 + fourthTurnTime;		//12.2


		if(timeF < startPauseTime){
		}else if (timeF < firstForwardTime) {
			//hkdrive.goForwardFacing(forwardVoltage, 0.0);
			liftMani.flipMouth();	SmartDashboard.putString("auto move", "Going forward");
			hkdrive.goForwardVoltage(forwardVoltage);
			liftMani.elevatorLift(liftMani.switchHeight);		//bring the cube "up"
		}else if(timeF < firstTurnTime) {
			hkdrive.autoTurn(turnAngle);
			SmartDashboard.putString("auto move", "turning");
			//hkdrive.turnVoltage(turnVoltage);
		}else if(timeF < secondForwardTime) {
			//hkdrive.goForwardFacing(forwardVoltage, angleToOurSwitchPlate);
			SmartDashboard.putString("auto move", "Going forward");
			hkdrive.goForwardVoltage(forwardVoltage);
		}else if(timeF < secondTurnTime) {
			hkdrive.autoTurn(0.0);
			SmartDashboard.putString("auto move", "turning");
			//hkdrive.turnVoltage(-turnVoltage);
		}else if(timeF < forwardToSwitchTime) {
			//hkdrive.goForwardFacing(forwardVoltage, -angleToOurSwitchPlate);
			hkdrive.goForwardVoltage(forwardVoltage);
		}else if (timeF < dipTime){
			hkdrive.stop();										//stop driving
			liftMani.dipMouth();								//let the mouth "Dip"
		}else if(timeF < spitTime){
			mouthMani.spit();									//spit out powercube
		}else if(timeF < backFromSwitchTime){
			hkdrive.goForwardVoltage(-forwardVoltage);
			liftMani.elevatorLift(liftMani.bottomHeight);
		}else if(timeF < thirdTurnTime) { 
			hkdrive.autoTurn(-turnAngle);
		}else if(timeF < backFromTurnTime) { 
			hkdrive.goForwardVoltage(forwardVoltage);
		}else if(timeF < fourthTurnTime) { 
			hkdrive.autoTurn(0.0);
		}else if(timeF < forwardTimeToCubes) { 
			hkdrive.goForwardVoltage(forwardVoltage);
			mouthMani.eat();
		}else{
			hkdrive.stop();										//stop driving
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

		if(getSwitchPlateSide() == 1) {
			return -90.0;					//turn left
		}else {
			return 90.0;					//turn right
		}
	}

	//method that gets an int for which side is our plate on the Switch, 1 = left, 3 = right -Marl
	public int getSwitchPlateSide(){

		String gameData = DriverStation.getInstance().getGameSpecificMessage().substring(0,1);

		SmartDashboard.putString("SwitchSide", gameData);

		if(gameData.equals("L")) return 1;
		else if (gameData.equals("R")) return 3;
		else return -2;

	}

	//method that gets an int for which side is our plate on the Scale, 1 = left, 3 = right -Marl
	public int getScalePlateSide(){

		String gameData = DriverStation.getInstance().getGameSpecificMessage().substring(1,2);

		SmartDashboard.putString("ScaleSide", gameData);

		if(gameData.equals("L")) return 1;
		else if (gameData.equals("R")) return 3;
		else return -2;

	}

	enum POSITION{
		L, C, R;

		public static POSITION valueOf(char charAt) {
			if(charAt == 'L')
				return POSITION.L;
			else if(charAt == 'R')
				return POSITION.R;
			else
				return POSITION.C;
		}

		public static POSITION valueOf(int currentPosition) {
			if(currentPosition == 1)
				return POSITION.L;
			else if(currentPosition == 2)
				return POSITION.R;
			else
				return POSITION.C;
		}

	}
	
	/*
	public POSITION getScalePlateSide(){

		String gameData = DriverStation.getInstance().getGameSpecificMessage();

		return POSITION.valueOf(gameData.charAt(1));
	}

	 */

	

	public void checkLed(){

		double matchTime = DriverStation.getInstance().getMatchTime();
		LedStrip.ledAllianceColor();

		
		if(matchTime < 30.0){
			LedStrip.partyTime();			
		}
	}
}

