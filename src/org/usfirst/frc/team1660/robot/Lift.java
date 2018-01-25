package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift {

	
	private Joystick manipStick;
	private WPI_TalonSRX liftMotor;
	private static final int kLiftMotorChannel = 4;//temp 7	
	
	
	public Lift(Joystick manipStick) {
		this.manipStick = manipStick;
	}
	
	public void liftInit() {
		liftMotor = new WPI_TalonSRX(kLiftMotorChannel); //A.K.A Elevator/Climb Manipulator
		liftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		/* set closed loop gains in slot0 - see documentation */
		liftMotor.selectProfileSlot(0, 0);
		liftMotor.config_kF(0, 0.2, 10);
		liftMotor.config_kP(0, 0.2, 10);
		liftMotor.config_kI(0, 0, 10);
		liftMotor.config_kD(0, 0, 10);
		/* set acceleration and vcruise velocity - see documentation */
		liftMotor.configMotionCruiseVelocity(15000, 10);
		liftMotor.configMotionAcceleration(6000, 10);
		
		/* zero the sensor */
		liftMotor.setSelectedSensorPosition(0, 0, 10);

	}

	
	//method to get the value from the armabot encoder- lakiera and pinzon
	public int getEncoder(){

		int x = liftMotor.getSelectedSensorPosition(0);
		SmartDashboard.putNumber("encoderPosition", x);
		return x;
	}


	/*----- BASIC ROBOT MOTION METHODS -----*/
	//method to lift up - mathew & marlahna
	public void elevatorLift(double setHeight) {
		
		if (manipStick.getRawButton(XboxButtons.A_BUTTON)) {
			
			SmartDashboard.putNumber("manipStick", 1);
			double leftYstick = -1.0 * manipStick.getY();
			double targetPos = leftYstick * 4096 * 10.0;
			liftMotor.set(ControlMode.MotionMagic, targetPos);
		}
		
		else
		{
			SmartDashboard.putNumber("manipStick", 2);
		}

	}

	//Basic method to climb down -Aldenis
	public void climbDown() {
		liftMotor.set(-1.0);
	}
	
	
	
}
