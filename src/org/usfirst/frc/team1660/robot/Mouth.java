package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Mouth {
	
	private Joystick maniStick;
	private WPI_TalonSRX mouthMotorLeft;
	private WPI_TalonSRX mouthMotorRight;
	private final int kMouthMotorChannelRight = 6;
	private final int kMouthMotorChannelLeft = 5;
	private final int kMouthLimitPort = 0;
	private DigitalInput limitSwitchMouth;
	private boolean isUpFlag;
	
	double speed = 0.5;
	
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
		mouthMotorLeft.set(speed);
		mouthMotorRight.set(-speed);
	}
	
	//eat method by Kwaku, does takes in the power cube  
	public void eat(){
	  mouthMotorLeft.set(-speed);
	  mouthMotorRight.set(speed);
	}
	
	//not moving method by Mal Einstein, method to hold the box
	public void shutUp(){
	  mouthMotorLeft.set(0);
	  mouthMotorRight.set(0);
	}
	
	
	//check eat/spit/mouth method
	public void checkEatSpit(){
		if(maniStick.getRawButton(XboxButtons.A_BUTTON) == true  ){
			eat();
		}
		else if(maniStick.getRawButton(XboxButtons.B_BUTTON)== true){
			spit();
		}else {
			shutUp();
		}
	}
	
	
	//check limit switch value and return it (accessor) -kwaku
	public boolean isPowercube() {
		boolean isPC = this.limitSwitchMouth.get();
		SmartDashboard.putBoolean("isPC?", isPC);
		return isPC;
	}
	
}
