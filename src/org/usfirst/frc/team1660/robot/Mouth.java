package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Mouth {
	
	private Joystick maniStick;
	private WPI_TalonSRX mouthMotorLeft;
	private WPI_TalonSRX mouthMotorRight;
	private final int kMouthMotorChannelRight = 5;
	private final int kMouthMotorChannelLeft = 6;
	private final int kMouthLimitPort = 0;
	private DigitalInput limitSwitchMouth;
	private boolean isUpFlag;
	
	
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
	public void checkEatSpit(){
		if(maniStick.getRawButton(XboxButtons.A_BUTTON) == true  ){
			eat();
		}
		if(maniStick.getRawButton(XboxButtons.B_BUTTON)== true){
			spit();
		}
	}
	
	
	//check limit switch value and return it (accessor) -kwaku
	public boolean isPowercube() {
		boolean isPC = this.limitSwitchMouth.get();
		SmartDashboard.putBoolean("isPC?", isPC);
		return isPC;
	}
	
}
