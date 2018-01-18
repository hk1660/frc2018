/*
 * Code for the Harlem Knights (FRC 1660) Robot for 2018
 */

package org.usfirst.frc.team1660.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class Robot<m_robotDrive> extends IterativeRobot {

	/*----DECLARED GLOBAL VARIABLES-------*/
	private static final int kFrontLeftChannel = 2;
	private static final int kBackLeftChannel = 3;
	private static final int kFrontRightChannel = 1;
	private static final int kBackRightChannel = 0;
	private static final int kSwitchBoi = 5; 			//lol we can switch the name l8r
	private static final int kClimbMotorChannel = 4;
	private static final int kJoystickChannel = 0;

	private MecanumDrive m_robotDrive;
	private Joystick m_stick1;
	private AHRS navx;

	WPI_TalonSRX climbMotor = new WPI_TalonSRX(kClimbMotorChannel);
	WPI_TalonSRX spitMotor = new WPI_TalonSRX(5);


	double roboAngle = 0.0;
	double zeroedYawPoint = 0.0;

	//JOYSTICK
	final int A_BUTTON = 1;
	final int B_BUTTON = 2;
	final int X_BUTTON = 3;
	final int Y_BUTTON = 4;
	final int LB_BUTTON = 5;
	final int RB_BUTTON = 6;
	final int BACK_BUTTON = 7;
	final int START_BUTTON = 8;
	final int LEFT_JOY_BUTTON = 9;
	final int RIGHT_JOY_BUTTON = 10;
	final int LEFT_X_AXIS = 0;
	final int LEFT_Y_AXIS = 1;
	final int LT_AXIS = 2;
	final int RT_AXIS = 3;
	final int RIGHT_X_AXIS = 4;
	final int RIGHT_Y_AXIS = 5;
	final int POV_UP = 0;
	final int POV_LEFT = 270;
	final int POV_DOWN = 180;
	final int POV_RIGHT = 90;

	Joystick driverStick = new Joystick(0);
	Joystick manipStick = new Joystick(1);
	private Joystick m_stick;
	
    public void robotInit() {

    	 WPI_TalonSRX climbMotor = new WPI_TalonSRX(kClimbMotorChannel);
    	 //A.K.A Climb Manipulator
		WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
		WPI_TalonSRX backLeft = new WPI_TalonSRX(kBackLeftChannel);
		WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);
		WPI_TalonSRX backRight = new WPI_TalonSRX(kBackRightChannel);


		try {
			navx = new AHRS(SPI.Port.kMXP); //navX-MXP initialized with (SPI, I2C, TTL UART) and USB //http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

		// Invert the left side motors.
		// You may need to change or remove this to match your robot.
		frontLeft.setInverted(true);
		backLeft.setInverted(true);

		m_robotDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

		m_stick1 = new Joystick(kJoystickChannel);
		
		
	}


	
	
	public void climbDown1() {
	climbMotor.set(-1.0);
		
	}




	/* AUTONOMOUS MODE */
	public void autonomousInit() {
		//autocode goes here @AmadouGamby & @marlahna
		// timer.reset(); // Resets the timer to 0
	     //timer.start(); // Start counting
	}	
	public void autonomousPeriodic() {

	}

	/* TELEOP MODE */ 
	public void teleopInit() { 
		
		
	}
	public void teleopPeriodic() {
		// Use the joystick X axis for lateral movement, Y axis for forward
		// movement, and Z axis for rotation.
		m_robotDrive.driveCartesian(m_stick1.getX(), m_stick1.getY(),
				m_stick1.getZ(), 0.0);
	}





	/* CUSTOM METHODS */

	public int getCurrentAngle(){
		int moddedAngle = Math.floorMod((int)navx.getAngle(), 360);
		int moddedZeroedYawPoint = Math.floorMod((int)zeroedYawPoint, 360);
		int modAngle = Math.floorMod((moddedAngle - moddedZeroedYawPoint), 360);

		SmartDashboard.putNumber("modAngle", modAngle);
		return modAngle;
	}
	
	// Aldenis
	

	
	
	
	//Robot Climbs down -Aldenis 
	public void climbDown() {
		climbMotor.set(-1.0);
	}

	
	// Robot spits out a PowerCube -Kwaku Boafo
	public void spit(){
		spitMotor.set(1.0);
	}





	
	
	
	//method to get the value from the armabot encoder- lakiera and pinzon
	public int getEncoder(){
		return 0;
		
		
		
		
	}
	
}
	
	
	


