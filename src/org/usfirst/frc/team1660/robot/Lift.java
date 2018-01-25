package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift {

	private WPI_TalonSRX liftMotor;
	private Joystick manipStick;
	private int LIFT_AXIS = XboxButtons.LEFT_Y_AXIS;

	private static final int kLiftMotorChannel = 7;
	private static final int kSlotIdx = 0;
	private static final int kPidIdx = 0;
	private static final int ktimeout = 10; //number of ms to update closed-loop control
	private static final double kF = 0.2;
	private static final double kP = 0.2;
	private static final double kI = 0.0;
	private static final double kD = 0.0;
		

	public Lift(Joystick manipStick) {
		this.manipStick = manipStick;
	}

	public void liftInit() {

		liftMotor = new WPI_TalonSRX(kLiftMotorChannel); //A.K.A Elevator/Climb Manipulator
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

	}


	//method to get the value from the armabot encoder- lakiera and pinzon
	public int getEncoder(){

		int x = liftMotor.getSelectedSensorPosition(kPidIdx);
		SmartDashboard.putNumber("liftHeight", x);
		return x;
	}


	//method to lift up (With Joystick) - mathew & marlahna
	public void checkElevatorLift() {

		double thresh = 0.1;
		double liftJoyValue = manipStick.getRawAxis(LIFT_AXIS);
		SmartDashboard.putNumber("Lift Axis", liftJoyValue);

		if (Math.abs(liftJoyValue) > thresh) {
			liftMotor.set(ControlMode.PercentOutput, liftJoyValue);
		} 

	}

	//method to lift up (to a SET Position) -mathew & marlahna
	public void elevatorLift(double setHeight) {

		SmartDashboard.putNumber("setHeight", setHeight);
		liftMotor.set(ControlMode.MotionMagic, setHeight); 
	}


	//Basic method to climb down -Aldenis
	public void climbDown() {
		liftMotor.set(-1.0);
	}



}
