package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift {

	private WPI_TalonSRX liftMotor;
	private Joystick maniStick;
	private DigitalInput limitLiftTop = new DigitalInput(RobotMap.LIFT_LIMIT_TOP_CHANNEL);
	private AnalogInput limitLiftBottom = new AnalogInput(RobotMap.LIFT_LIMIT_BOTTOM_CHANNEL);

	//Compressor comp = new Compressor(RobotMap.COMPRESSOR_PORT);
	DoubleSolenoid flipDipPistons;
	DoubleSolenoid liftLockPistons;
	private static final int kSlotIdx = 0;
	private static final int kPidIdx = 0;
	private static final int ktimeout = 10; //number of ms to update closed-loop control
	private static final double kF = 0.2;
	private static final double kP = 0.3;
	private static final double kI = 0.0;
	private static final double kD = 0.0;
	
	private static final double ANALOG_VOLTAGE_THRESHOLD = 4.0;

	private static boolean isDip = false;
	private static boolean isLock = false;
	//boolean enabled = comp.enabled();
	//boolean pressureSwitch = comp.getPressureSwitchValue();
	//double current = comp.getCompressorCurrent();

	//int rawPerRev = 13300 + 2347 - 700;  //pos numbers go up in air
	//int rawPerRev = 8200;  //pos numbers go up in air

	//height setpoints in inches
	double bottomHeight = 0.0;
	double topHeight = 73000.0;
	double switchHeight = 66000.0;
	double exchangeHeight = 4748.0;
	double tier2Height = 26280.0;
	double liftTargetHeight = -1.0;
	double pullUpHeight = 28000.0;

	int reachHeight = 68000; 			//raw units


	boolean manualFlag;
	boolean climbFlag;
	private boolean disengageMotorFlag = false;

	public Lift(Joystick maniStick) {
		this.maniStick = maniStick;
	}

	public void liftInit() {

		flipDipPistons = new DoubleSolenoid(RobotMap.FLIP_PORT, RobotMap.DIP_PORT);
		liftLockPistons = new DoubleSolenoid(RobotMap.LOCK_PORT, RobotMap.UNLOCK_PORT);

		liftMotor = new WPI_TalonSRX(RobotMap.LIFT_MOTOR_CHANNEL); //A.K.A Elevator/Climb maniulator

		disengageMotorFlag = false;
		liftMotor.setInverted(true);

		liftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPidIdx, ktimeout);

		// set closed loop gains in desired slot
		liftMotor.selectProfileSlot(kSlotIdx, kPidIdx);
		liftMotor.config_kF(kSlotIdx, kF, ktimeout);
		liftMotor.config_kP(kSlotIdx, kP, ktimeout);
		liftMotor.config_kI(kSlotIdx, kI, ktimeout);
		liftMotor.config_kD(kSlotIdx, kD, ktimeout);

		// set acceleration and vcruise velocity
		liftMotor.configMotionCruiseVelocity(15000, ktimeout);
		liftMotor.configMotionAcceleration(6000, ktimeout);

		// zero the sensor
		liftMotor.setSelectedSensorPosition(0, kPidIdx, ktimeout);

		manualFlag = false;
		climbFlag = false;

		this.setEncoderZero();
		this.elevatorLift(0);

	}


	//method to get the value from the encoder- lakiera and pinzon (Black side faces LEFT, Silver side faces RIGHT)
	public int getEncoder(){

		int x = liftMotor.getSelectedSensorPosition(kPidIdx);
		SmartDashboard.putNumber("liftHeight", x);
		return x;
	}

	//joystick method to zero encoder -pinzon & lakiera
	public void checkEncoderZero() {
		if(maniStick.getRawButton(RobotMap.ZERO_ENCODER_BUTTON)==true) {
			this.setEncoderZero();
			SmartDashboard.putString("EncZeroWHO?", "Button");
		}


	}

	//base method to set the encoder value to zero -pinzon & lakiera
	public void setEncoderZero() {
		liftMotor.setSelectedSensorPosition(0, this.kPidIdx, this.ktimeout);
	}

	//method to convert height in inches off the ground to raw units -pinzon & lakiera
	/*public int convert(double height) {
		double diameter = 1.66; //inches
		double circumference = diameter * Math.PI;	//inches/rev
		double revs = height / circumference;
		double raw =  revs * this.rawPerRev;

		return (int)raw;
	}
	 */


	//joystick method to make the lift move to a specific height -pinzon & lakiera & Mal
	public void  checkLiftPoints() {

		/*
		if (maniStick.getPOV()==RobotMap.LB_BUTTON) {
			manualFlag = false;
			liftTargetHeight = this.convert(topHeight);
		}
		 */

		int povVal = maniStick.getPOV();

		if (povVal == (int)(RobotMap.LIFT_BOTTOM_HEIGHT_POV)) {
			manualFlag = false;
			SmartDashboard.putString("POV", "down");
			liftTargetHeight = this.bottomHeight;
		}
		if (povVal==(int)RobotMap.LIFT_SWITCH_HEIGHT_POV) {
			manualFlag = false;
			SmartDashboard.putString("POV", "switch");
			liftTargetHeight = this.switchHeight;
		}
		if (povVal==(int)RobotMap.LIFT_EXCHANGE_HEIGHT_POV) {
			manualFlag = false;
			SmartDashboard.putString("POV", "exchange");
			liftTargetHeight = this.exchangeHeight;
		}
		if (povVal==(int)RobotMap.LIFT_TIER2_HEIGHT_POV) {
			manualFlag = false;
			SmartDashboard.putString("POV", "tier2");
			liftTargetHeight = this.tier2Height;
		}		
		if(manualFlag == false) {
			elevatorLift(liftTargetHeight);
		}
		SmartDashboard.putNumber("Lift POV?", povVal);
		SmartDashboard.putNumber("LiftTargetHeight", liftTargetHeight);
	}

	/* to shoot up climber at push of a button -@mathew */
	public void checkClimb() { 

		if(maniStick.getRawButton(RobotMap.REACH_BUTTON) == true )	{
			manualFlag = false;
			SmartDashboard.putString("Climb?", "Raising Up!");
			liftTargetHeight = this.reachHeight;
			
		}
		else if (maniStick.getRawButton(RobotMap.PULL_UP_BUTTON) == true ) {
			manualFlag = false;
			SmartDashboard.putString("Climb?", "Robot in the Air!");
			liftTargetHeight = this.pullUpHeight;
			climbFlag = true;
		}

	}




	//Method to move the elevator up & down- mathew & marlahna
	public void checkManualLift() {

		double thresh = 0.1;
		double liftJoyValue = -maniStick.getRawAxis(RobotMap.LIFT_AXIS); //joystic val negative when go up we switched 
		boolean botVal = limitLiftBottom.getVoltage() > ANALOG_VOLTAGE_THRESHOLD;
		boolean topVal = limitLiftTop.get();
		SmartDashboard.putNumber("Lift Axis", liftJoyValue);
		SmartDashboard.putBoolean("limit top value", topVal);
		SmartDashboard.putBoolean("limit bottom value", botVal);

		//re-zero lift if you hit the bottom
		if (botVal) {
			SmartDashboard.putString("Limits", "Bottom limit has been hit");
			this.setEncoderZero();
			SmartDashboard.putString("EncZeroWHO?", "LS bottom");
		}

		//check motion if the joystick axis is being pushed
		if (Math.abs(liftJoyValue) > thresh && !disengageMotorFlag) {
			manualFlag = true;
			if(botVal && liftJoyValue < 0) {	//don't move down if at bottom
				SmartDashboard.putString("Limits", "STOP! Bottom limit has been hit");
				liftMotor.set(ControlMode.PercentOutput, 0.0);
			} else if(topVal && liftJoyValue > 0) {		//don't move up if at top
				SmartDashboard.putString("Limits", "STOP! Top limit has been hit");
				liftMotor.set(ControlMode.PercentOutput, 0.0);
			} else {							//move up & down with joystick axis
				liftMotor.set(ControlMode.PercentOutput, liftJoyValue);
			}

			//shut off lift when you stop pushing the axis
		} else if(manualFlag && !disengageMotorFlag) { //turn off motor if stopped pushing
			liftMotor.set(ControlMode.PercentOutput, 0.0);
		}

		SmartDashboard.putBoolean("liftManualFlag", manualFlag);
		elevatorLift(liftJoyValue);
	}


	/*	WIP	*/
	//method to lift up (to a SET Position) -mathew & marlahna
	public void elevatorLift(double setHeight) {

		SmartDashboard.putNumber("setHeight", setHeight);
		Boolean botVal = limitLiftBottom.getVoltage() > ANALOG_VOLTAGE_THRESHOLD;
		Boolean topVal = limitLiftTop.get();
		//double joy = maniStick.getRawAxis(RobotMap.LIFT_AXIS);

		if(botVal ) {
			SmartDashboard.putString("Limits", "STOP! Bottom limit has been hit");
			//liftMotor.set(ControlMode.PercentOutput, 0.0);
			this.setEncoderZero();
			SmartDashboard.putString("EncZeroWHO?", "Button in EL");
		}
		else if(topVal) {
			SmartDashboard.putString("Limits", "STOP! Top limit has been hit");
			//liftMotor.set(ControlMode.PercentOutput, 0.0);
		}


		if(manualFlag == true && !disengageMotorFlag) {  //when touching the joystick
			double joyVal = setHeight;
			SmartDashboard.putNumber("manual", setHeight);
			liftMotor.set(ControlMode.PercentOutput,joyVal);
		}
		else if (manualFlag == false && !disengageMotorFlag) {//when touching the POV or AUTO\\
			SmartDashboard.putNumber("magic motion", setHeight);
			liftMotor.set(ControlMode.MotionMagic,setHeight);
		}


	}

	/*method to turn compressor on and off -nana
	public void checkCompressor(){ 

		if(maniStick.getRawButton(RobotMap.COMPRESSOR_ON_BUTTON) == true){
			this.compressorOn();
			SmartDashboard.putString("Compressor: ", "ON-button");
		}
		else if(maniStick.getRawButton(RobotMap.COMPRESSOR_OFF_BUTTON) == true){
			this.compressorOff();
			SmartDashboard.putString("Compressor: ", "OFF-button");
		}
	} */


	/* methods to flip (up) and dip (down) the mouth -Aldenis */
	public void flipMouth() {
		flipDipPistons.set(DoubleSolenoid.Value.kForward);
		isDip = false;
	}

	public void dipMouth() {
		flipDipPistons.set(DoubleSolenoid.Value.kReverse);	
		isDip = true;
	}

	/* Check if mouth is flipped already? */
	public boolean isDipped() {
		return isDip;
	}

	/* method to check if flip or dip buttons are pressed -mohamed */
	public void checkFlipDip() {
		if(maniStick.getRawAxis(RobotMap.DIP_AXIS) > 0.5) {
			dipMouth();
			SmartDashboard.putString("FlipDip", "DIP");
		}
		else if(maniStick.getRawAxis(RobotMap.FLIP_AXIS) > 0.5) {
			flipMouth();
			SmartDashboard.putString("FlipDip", "FLIP");
		} else {
			SmartDashboard.putString("FlipDip", "NONE");
		}
	}
	//method for the lock and unlock mechansm to work
	public void lock() { 
		liftLockPistons.set(DoubleSolenoid.Value.kForward);	
		isLock = true;
	}

	public void unlock() {
		liftLockPistons.set(DoubleSolenoid.Value.kReverse);	
		isLock = false;
	}
	//method to check if lock & unlock button is pressed
	public void checkLockUnlock() {
		if(maniStick.getRawButton(RobotMap.LOCK_BUTTON) == true && climbFlag == true) {
			lock();
			SmartDashboard.putString("lock/unlock", "locked");
		}
		else if(maniStick.getRawButton(RobotMap.UNLOCK_BUTTON) == true) {
			unlock();
			SmartDashboard.putString("lock/unlock", "unlocked");
		} else {
			SmartDashboard.putString("lock/unlock", "NONE");
		}
	}

	public void checkDisengageMotor() {
		if(maniStick.getRawButton(RobotMap.RELAX_MOTOR_BUTTON)==true) {
			disengageMotorFlag= true;

		}
	}

	public void disengageMotor() {
		if(disengageMotorFlag == true) {
			//liftMotor.set(ControlMode.PercentOutput,0.0);
			//liftMotor.disable();
			liftMotor.set(ControlMode.Disabled,0);

			SmartDashboard.putString("motor enabled?", "NO");
		}
	}

	/* basic compressor functionality methods	-Aldenis 
	public void compressorOn(){
		this.comp.setClosedLoopControl(true);
		SmartDashboard.putString("compressorStatus", "is on");
	}
	public void compressorOff(){
		this.comp.setClosedLoopControl(false);
		SmartDashboard.putString("compressorStatus", "is off");
	}	

	 */

	/* basic compressor functionality methods	
	public void compressorOn(){
		//this.compressorRelay.set(Relay.Value.kForward);
		SmartDashboard.putString("compressorStatus", "is on");
	}
	public void compressorOff(){
		//this.compressorRelay.set(Relay.Value.kOff);
		SmartDashboard.putString("compressorStatus", "is off");
	}

	 */


	
	
}
