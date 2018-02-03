package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift {

	private WPI_TalonSRX liftMotor;
	private Joystick maniStick;
	private DigitalInput limitLiftTop = new DigitalInput(RobotMap.LIFT_LIMIT_TOP_CHANNEL);
	private DigitalInput limitLiftBottom = new DigitalInput(RobotMap.LIFT_LIMIT_BOTTOM_CHANNEL);

	private static final int kSlotIdx = 0;
	private static final int kPidIdx = 0;
	private static final int ktimeout = 10; //number of ms to update closed-loop control
	private static final double kF = 0.2;
	private static final double kP = 0.3;
	private static final double kI = 0.0;
	private static final double kD = 0.0;

	int rawPerRev = -13300 - 2347 + 700;  //neg numbers go up in air
	//int rawPerRev = -8200;  //neg numbers go up in air

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
		if (maniStick.getPOV()==RobotMap.LB_BUTTON) {
			manualFlag = false;
			liftTargetHeight = this.convert(topHeight);
		}
		if (maniStick.getPOV()==RobotMap.LIFT_BOTTOM_HEIGHT_POV) {
			manualFlag = false;
			liftTargetHeight = this.convert(bottomHeight);
		}
		if (maniStick.getPOV()==RobotMap.LIFT_SWITCH_HEIGHT_POV) {
			manualFlag = false;
			liftTargetHeight = this.convert(switchHeight);
		}
		if (maniStick.getPOV()==RobotMap.LIFT_EXCHANGE_HEIGHT_POV) {
			manualFlag = false;
			liftTargetHeight = this.convert(exchangeHeight);
		}
		if (maniStick.getPOV()==RobotMap.LIFT_TIER2_HEIGHT_POV) {
			manualFlag = false;
			liftTargetHeight = this.convert(tier2Height);
		}		
		if(manualFlag == false) {
			elevatorLift(liftTargetHeight);
		}
		SmartDashboard.putNumber("LiftTargetHeight", liftTargetHeight);
	}

	//Method to move the elevator up & down- mathew & marlahna
	public void checkElevatorLift() {

		double thresh = 0.1;
		double liftJoyValue = -maniStick.getRawAxis(RobotMap.LIFT_AXIS); //joystic val negative when go up we switched 
		SmartDashboard.putNumber("Lift Axis", liftJoyValue);

		/*  */
		Boolean botVal = limitLiftBottom.get();
		Boolean topVal = limitLiftTop.get();
		
		SmartDashboard.putBoolean("limit top value", topVal);
		SmartDashboard.putBoolean("limit bottom value", botVal);
		
		SmartDashboard.putNumber("lift joy value", liftJoyValue);

		
		if (Math.abs(liftJoyValue) > thresh) {
			manualFlag = true;
			if(botVal && liftJoyValue > 0) {
				SmartDashboard.putString("Limits", "STOP! Bottom limit has been hit");
				liftMotor.set(ControlMode.PercentOutput, 0.0);
				this.setEncoderZero();
			}
			else if(topVal && liftJoyValue < 0) {
				SmartDashboard.putString("Limits", "STOP! Top limit has been hit");
				liftMotor.set(ControlMode.PercentOutput, 0.0);
			}
			else if(manualFlag == true) {
				SmartDashboard.putString("Limits", "In between Limits");
				liftMotor.set(ControlMode.PercentOutput, liftJoyValue);
			}
		} else { 
			if(manualFlag == true) {
				liftMotor.set(ControlMode.PercentOutput, 0.0);
			}

		}

		SmartDashboard.putBoolean("liftManualFlag", manualFlag);
	}


	/*	WIP	*/
	//method to lift up (to a SET Position) -mathew & marlahna
	public void elevatorLift(double setHeight) {

		SmartDashboard.putNumber("setHeight", setHeight);
		Boolean botVal = limitLiftBottom.get();
		Boolean topVal = limitLiftTop.get();
		if(botVal) {
			SmartDashboard.putString("Limits", "STOP! Bottom limit has been hit");
			liftMotor.set(ControlMode.PercentOutput, 0.0);
		}
		else if(topVal) {
			SmartDashboard.putString("Limits", "STOP! Top limit has been hit");
			liftMotor.set(ControlMode.PercentOutput, 0.0);
		}
		else if(manualFlag == true) {
			liftMotor.set(ControlMode.MotionMagic, setHeight);
		}
	}

	//Basic method to climb down -Aldenis
	public void climbDown() {
		liftMotor.set(-1.0);
	}



}
