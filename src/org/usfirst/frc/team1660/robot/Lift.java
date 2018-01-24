package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift {

	
	private Joystick manipStick;
	private WPI_TalonSRX liftMotor;
	private static final int kLiftMotorChannel = 7;	
	
	
	public Lift(Joystick manipStick) {
		this.manipStick = manipStick;
	}
	
	public void liftInit() {
		liftMotor = new WPI_TalonSRX(kLiftMotorChannel); //A.K.A Elevator/Climb Manipulator		
	}

	
	//method to get the value from the armabot encoder- lakiera and pinzon
	public int getEncoder(){

		int x = liftMotor.getSelectedSensorPosition(0);
		SmartDashboard.putNumber("encoderPosition", x);
		return x;
	}


	/*----- BASIC ROBOT MOTION METHODS -----*/

	//Basic method to climb down -Aldenis
	public void climbDown() {
		liftMotor.set(-1.0);
	}
	
	
	
}
