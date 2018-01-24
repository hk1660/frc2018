package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Joystick;

public class Mouth {
	
	private Joystick manipStick;
	private WPI_TalonSRX mouthMotorLeft;
	private WPI_TalonSRX mouthMotorRight;
	private final int kMouthMotorChannelRight = 4;
	private final int kMouthMotorChannelLeft = 5;
	private final int kMouthLimitPort = 0;
	private DigitalInput limitSwitchMouth;
	private boolean isUp;
	
	
	public Mouth(Joystick manipStick) {
		this.manipStick = manipStick;
	}


	public void mouthInit() {
		mouthMotorLeft = new WPI_TalonSRX(kMouthMotorChannelLeft);
		mouthMotorRight = new WPI_TalonSRX(kMouthMotorChannelRight);
		limitSwitchMouth = new DigitalInput(kMouthLimitPort);
	}
	


	// Basic method for robot to spit out a PowerCube -Kwaku Boafo
	public void spit(){
		mouthMotorLeft.set(1.0);
		mouthMotorRight.set(-1.0);
	}
	
	//eat method
	
	
	
	//check eat/spit/mouth method
	
	
	
	//check limit switch value and return it (accessor)
	
	
}
