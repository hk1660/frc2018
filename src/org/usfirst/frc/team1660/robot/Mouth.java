package org.usfirst.frc.team1660.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Mouth {
	
	private Joystick maniStick;
	private WPI_TalonSRX mouthMotorLeft;
	private WPI_TalonSRX mouthMotorRight;
	private DigitalInput limitSwitchMouth;
	private boolean isUpFlag;
	
	double mouthSpeed = 0.8;
	
	public Mouth(Joystick maniStick) {
		this.maniStick = maniStick;
	}


	public void mouthInit() {
		mouthMotorLeft = new WPI_TalonSRX(RobotMap.MOUTH_LEFT_CHANNEL);
		mouthMotorRight = new WPI_TalonSRX(RobotMap.MOUTH_RIGHT_CHANNEL);
	//	limitSwitchMouth = new DigitalInput(RobotMap.MOUTH_LIMITER_CHANNEL);
	}
	

	// Basic method for robot to spit out a PowerCube -Kwaku Boafo
	public void spit(){
		mouthMotorLeft.set(mouthSpeed);
		mouthMotorRight.set(-mouthSpeed);
	}
	
	//eat method by Kwaku, does takes in the power cube  
	public void eat(){
	  mouthMotorLeft.set(-mouthSpeed);
	  mouthMotorRight.set(mouthSpeed);
	}
	
	//not moving method by Mal, method to hold the box
	public void shutUp(){
	  mouthMotorLeft.set(0);
	  mouthMotorRight.set(0);
	}
	
	
	//check eat/spit/mouth method
	public void checkEatSpit(){
		if(maniStick.getRawButton(RobotMap.EAT_BUTTON) == true  ){
			eat();
		}
		else if(maniStick.getRawButton(RobotMap.SPIT_BUTTON)== true){
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
