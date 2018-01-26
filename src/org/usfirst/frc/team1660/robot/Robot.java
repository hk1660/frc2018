/*
 * Code for the Harlem Knights (FRC 1660) Robot for 2018
 * website: www.hk1660.com
 * online repository: www.github.com/hk1660/frc2018
 */

package org.usfirst.frc.team1660.robot;

/*-----IMPORTED LIBRARIES-----*/

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;


public class Robot<m_robotDrive> extends IterativeRobot {

	Joystick driverStick = new Joystick(0);
	Joystick maniStick = new Joystick(1);
	HKDrive hkdrive = new HKDrive(driverStick);
	Lift liftMani = new Lift(maniStick);
	Mouth mouthMani = new Mouth(maniStick);
	//Lidar laser = new Lidar();
	Lidar2 laser2 = new Lidar2(I2C.Port.kMXP, (byte) 0x62);
	SendableChooser strategy = new SendableChooser();
	


	/*----- REQUIRED FRC MAIN METHODS -----*/
	public void robotInit() {
		hkdrive.driveInit();		//Initialize the HKDrive speed controllers
		liftMani.liftInit();
		mouthMani.mouthInit();
		laser2.setup();
		
		/* Auto mode strategies */
		strategy.addDefault("Red Alliance: Left Switch Pad", new Integer(1));
/*		strategy.addObject("Red Alliance: Right Switch Pad", new Integer(2));
		strategy.addObject("Blue Alliance: Left Switch Pad", new Integer(3));
		strategy.addObject("Blue Alliance: Right Switch Pad", new Integer(4));
		SmartDashboard.putData("strategy selector", strategy); */
	}

	//AUTONOMOUS MODE
	
	/* Autonomous Stuff \o/ -Khalil */
	public void autonomousInit() {
		Timer timerAuto = new Timer();
		timerAuto.start();
		//int currentStrategy = (int) strategy.getSelected(); 

		hkdrive.resetAngle();
		while(isAutonomous() && isEnabled()){ 

			double timerA = timerAuto.get();
			SmartDashboard.putNumber("AutoTimer",timerA);
			
/* 			Strategies need to be made before this is continued further
 * 
 * 			Odd: Left Pad
 * 			Even: Right Pad
 * 			1 & 2: Red Alliance 
 * 			3 & 4: Blue Alliance
 * 
 * 			if(currentStrategy == 1) {
 * 		} else if (currentStrategy == 2){
 * 		} else if (currentStrategy == 3){
 * 		} else if (currentStrategy == 4){
 * 		} 
*/	
			
		}
	}

	public void autonomousPeriodic() {

	}

	//TELEOP MODE
	public void teleopInit() { 
		//hkdrive.resetAngle();  //delete later
	}

	public void teleopPeriodic() {

		hkdrive.checkDriving();
		hkdrive.checkAutoTurn();
		hkdrive.getCurrentAngle();
		hkdrive.checkResetAngle();

		mouthMani.checkEatSpit();
		
		liftMani.getEncoder();
		liftMani.checkEncoderZero();
		liftMani.checkLiftPoints();
		liftMani.checkElevatorLift();
		
		laser2.getDistance();


	}



	/*------------------------- CUSTOM METHODS -------------------------*/

	/*----- AUTONOMOUS STRATEGY METHODS -----*/

}


