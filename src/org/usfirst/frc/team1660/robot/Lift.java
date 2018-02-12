package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
	private DigitalInput limitLiftBottom = new DigitalInput(RobotMap.LIFT_LIMIT_BOTTOM_CHANNEL);

	/*
	private static final int manualLiftMode = 1;
	private static final int manualLiftMode = 2;
	private static final int manualLiftMode = 3;
	 */


	Compressor comp = new Compressor(RobotMap.COMPRESSOR_CHANNEL);
	DoubleSolenoid dSolenoid = new DoubleSolenoid(1, 2);
	//Relay compressorRelay = new Relay(10);	//Temporary relay change later

	private static final int kSlotIdx = 0;
	private static final int kPidIdx = 0;
	private static final int ktimeout = 10; //number of ms to update closed-loop control
	private static final double kF = 0.2;
	private static final double kP = 0.3;
	private static final double kI = 0.0;
	private static final double kD = 0.0;

	private static boolean isDip = false;
	boolean enabled = comp.enabled();
	boolean pressureSwitch = comp.getPressureSwitchValue();
	double current = comp.getCompressorCurrent();

	//int rawPerRev = 13300 + 2347 - 700;  //pos numbers go up in air
	int rawPerRev = 8200;  //pos numbers go up in air

	//height setpoints in inches
	double bottomHeight = 0.0;
	double topHeight = 30.0;
	double switchHeight = 20.0;
	double exchangeHeight = 2.0;
	double tier2Height = 11.0;
	int liftTargetHeight = -1;

	boolean manualFlag;

	public Lift(Joystick maniStick) {
		this.maniStick = maniStick;
	}

	public void liftInit() {

		liftMotor = new WPI_TalonSRX(RobotMap.LIFT_MOTOR_CHANNEL); //A.K.A Elevator/Climb maniulator
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
		}


	}

	//base method to set the encoder value to zero -pinzon & lakiera
	public void setEncoderZero() {
		liftMotor.setSelectedSensorPosition(0, this.kPidIdx, this.ktimeout);
	}

	//method to convert height in inches off the ground to raw units -pinzon & lakiera
	public int convert(double height) {
		double diameter = 1.66; //inches
		double circumference = diameter * Math.PI;	//inches/rev
		double revs = height / circumference;
		double raw =  revs * this.rawPerRev;

		//SmartDashboard.putNumber(", value)

		return (int)raw;
	}



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
			SmartDashboard.putString("POV", "touching");
			liftTargetHeight = this.convert(bottomHeight);
		}
		if (povVal==(int)RobotMap.LIFT_SWITCH_HEIGHT_POV) {
			manualFlag = false;
			SmartDashboard.putString("POV", "touching");
			liftTargetHeight = this.convert(switchHeight);
		}
		if (povVal==(int)RobotMap.LIFT_EXCHANGE_HEIGHT_POV) {
			manualFlag = false;
			SmartDashboard.putString("POV", "touching");
			liftTargetHeight = this.convert(exchangeHeight);
		}
		if (povVal==(int)RobotMap.LIFT_TIER2_HEIGHT_POV) {
			manualFlag = false;
			SmartDashboard.putString("POV", "touching");
			liftTargetHeight = this.convert(tier2Height);
		}		
		if(manualFlag == false) {
			elevatorLift(liftTargetHeight);
		}
		SmartDashboard.putNumber("Lift POV?", povVal);
		SmartDashboard.putNumber("LiftTargetHeight", liftTargetHeight);
	}

	/* to shoot up climber at push of a button -@mathew */
	public void checkClimb() { 

		if(maniStick.getRawButton(RobotMap.CLIMB_UP_BUTTON) == true )	{
			manualFlag = false;
			SmartDashboard.putString("Climb?", "Raising Up!");
			liftTargetHeight = this.convert(topHeight);			
		}
		else if (maniStick.getRawButton(RobotMap.CLIMB_DOWN_BUTTON) == true ) {
			manualFlag = false;
			SmartDashboard.putString("Climb?", "Robot in the Air!");
			liftTargetHeight = this.convert(bottomHeight);
		}
			
	}




	//Method to move the elevator up & down- mathew & marlahna
	public void checkManualLift() {

		double thresh = 0.1;
		double liftJoyValue = -maniStick.getRawAxis(RobotMap.LIFT_AXIS); //joystic val negative when go up we switched 
		boolean botVal = limitLiftBottom.get();
		boolean topVal = limitLiftTop.get();
		SmartDashboard.putNumber("Lift Axis", liftJoyValue);
		SmartDashboard.putBoolean("limit top value", topVal);
		SmartDashboard.putBoolean("limit bottom value", botVal);

		//re-zero lift if you hit the bottom
		if (botVal) {
			SmartDashboard.putString("Limits", "Bottom limit has been hit");
			this.setEncoderZero();
		}

		//check motion if the joystick axis is being pushed
		if (Math.abs(liftJoyValue) > thresh) {
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
		} else if(manualFlag == true) { //turn off motor if stopped pushing
			liftMotor.set(ControlMode.PercentOutput, 0.0);
		}

		SmartDashboard.putBoolean("liftManualFlag", manualFlag);
		elevatorLift(liftJoyValue);
	}


	/*	WIP	*/
	//method to lift up (to a SET Position) -mathew & marlahna
	public void elevatorLift(double setHeight) {

		SmartDashboard.putNumber("setHeight", setHeight);
		Boolean botVal = limitLiftBottom.get();
		Boolean topVal = limitLiftTop.get();
		//double joy = maniStick.getRawAxis(RobotMap.LIFT_AXIS);

		if(botVal ) {
			SmartDashboard.putString("Limits", "STOP! Bottom limit has been hit");
			//liftMotor.set(ControlMode.PercentOutput, 0.0);
			this.setEncoderZero();
		}
		else if(topVal) {
			SmartDashboard.putString("Limits", "STOP! Top limit has been hit");
			//liftMotor.set(ControlMode.PercentOutput, 0.0);
		}


		if(manualFlag == true) {  //when touching the joystick
			double joyVal = setHeight;
			liftMotor.set(ControlMode.PercentOutput,joyVal);
		}
		else if (manualFlag == false) {//when touching the POV or AUTO
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
		dSolenoid.set(DoubleSolenoid.Value.kForward);
		isDip = false;
	}

	public void dipMouth() {
		dSolenoid.set(DoubleSolenoid.Value.kReverse);	
		isDip = true;
	}


	/* Check if mouth is flipped already? */
	public boolean isDipped() {
		return isDip;
	}

	public void checkDip() {
		if(maniStick.getRawAxis(RobotMap.DIP_AXIS) > 0.5) {
			dipMouth();
			SmartDashboard.putString("checkDip", "true");
		}

	}
	public void checkFlip() {
		if(maniStick.getRawAxis(RobotMap.FLIP_AXIS) > 0.5) {
			flipMouth();
			SmartDashboard.putString("checkFlip", "true");
		}
	}





	/* basic compressor functionality methods	-Aldenis */
	public void compressorOn(){
		this.comp.setClosedLoopControl(true);
		SmartDashboard.putString("compressorStatus", "is on");
	}
	public void compressorOff(){
		this.comp.setClosedLoopControl(false);
		SmartDashboard.putString("compressorStatus", "is off");
	}	

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
