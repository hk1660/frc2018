package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Joystick;

public class Mouth {
	
	private Joystick maniStick;
	private WPI_TalonSRX mouthMotorLeft;
	private WPI_TalonSRX mouthMotorRight;
	private final int kMouthMotorChannelRight = 5;
	private final int kMouthMotorChannelLeft = 6;
	private final int kMouthLimitPort = 0;
	private DigitalInput limitSwitchMouth;
	private boolean isUp;
	
	
	public Mouth(Joystick maniStick) {
		this.maniStick = maniStick;
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
	
	//eat method by Kwaku, does takes in the power cube  
	public void eat(){
	  mouthMotorLeft.set(-1.0);
	  mouthMotorRight.set(1.0);
	}
	
	
	//check eat/spit/mouth method
	
	
	
	//check limit switch value and return it (accessor)
	
	
}
