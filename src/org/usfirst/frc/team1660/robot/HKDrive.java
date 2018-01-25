package org.usfirst.frc.team1660.robot;

/*
 * Harlem Knights Drive class for:
 * 		+Mecanum Wheel drive
 * 		+Using TalsonSRX speed controllers
 * 		+With angles determined by NavX sensor
 * online repository: www.github.com/hk1660/frc2018
 */

/*-----IMPORTED LIBRARIES-----*/
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class HKDrive {

	/*----DECLARED GLOBAL VARIABLES-------*/

	//Drivetrain Declarations
	private WPI_TalonSRX frontLeft;
	private WPI_TalonSRX backLeft;
	private WPI_TalonSRX frontRight;
	private WPI_TalonSRX backRight;
	private MecanumDrive mecDrive;
	private AHRS navx;
	
	private static final int kFrontLeftChannel = 3;
	private static final int kBackLeftChannel = 7;//temp 4
	private static final int kFrontRightChannel = 2;
	private static final int kBackRightChannel = 1;
	
	double fieldAngleDifference = 0.0;
	double roboAngle = 0.0;
	double lastUsedAngle;
	boolean autoDriveFlag = false;	//automatic driving

	//Joystick fields
	private Joystick driverStick;
	private int FORWARD_AXIS = XboxButtons.LEFT_Y_AXIS;
	private int STRAFE_AXIS = XboxButtons.LEFT_X_AXIS;
	private int TURN_AXIS = XboxButtons.RIGHT_X_AXIS;
	
	
	public HKDrive(Joystick joy){
		this.driverStick = joy;
		
	}

	//HKDrive init method
	public void driveInit() {

		//Drivetrain Initializations
		frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
		backLeft = new WPI_TalonSRX(kBackLeftChannel);
		frontRight = new WPI_TalonSRX(kFrontRightChannel);
		backRight = new WPI_TalonSRX(kBackRightChannel);

		//we think the constructor switched the 3rd & 4th parameters
		mecDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

		//navx intialization
		try {
			navx = new AHRS(SPI.Port.kMXP); //navX-MXP initialized with (SPI, I2C, TTL UART) and USB //http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
	}


	//method to check joysticks for driving the robot -Nana B & Matthew W
	public void checkDriving() {

		double strafe = squareIt(driverStick.getRawAxis(STRAFE_AXIS)) ; // right and left on the left thumb stick?
		double forward = squareIt(driverStick.getRawAxis(FORWARD_AXIS));// up and down on left thumb stick?
		double turn = squareIt(driverStick.getRawAxis(TURN_AXIS));// right and left on right thumb stick


		//MECANUM -Malachi P
		if(autoDriveFlag == false ){

			//we think the parameters were wrong: S>T>F not F>S>T, actually... S>F>T
			mecDrive.driveCartesian(strafe, -forward, turn, 0);
			//mecDrive.driveCartesian(-strafe, -turn, -forward, getCurrentAngle());

			//Prints
			SmartDashboard.putNumber("move",	forward);
			SmartDashboard.putNumber("rotate",	turn);
			SmartDashboard.putNumber("strafe",	strafe);
		} 

		SmartDashboard.putBoolean("AutoDriving?", autoDriveFlag);

	}

	//method to square the joystick values to provide less sensitivity for small movements -Matthew W
	public double squareIt(double joy) {

		double squared = joy * joy;
		if(joy < 0) { squared *= -1; }
		return squared;
	}

	//joystick method to manually resets the robot to the field's orientation -Aldenis G
	public void checkResetAngle(){

		if (driverStick.getRawButton(XboxButtons.START_BUTTON)) {
			resetAngle();
		}
	}
	
	public void resetAngle() {
		fieldAngleDifference = navx.getAngle();
	}
	
	public void autoTurn(int futureAngle){

		//find correct speed to turn
		double desired_speed = autoTurnSpeed(futureAngle);

		//Ability to Strafe while maintaining the desired angle
		double strafeSpeedLeft= driverStick.getRawAxis(XboxButtons.LT_AXIS);
		double strafeSpeedRight = driverStick.getRawAxis(XboxButtons.RT_AXIS);
		double strafeSpeedActual;
		double minMotorSpeed = 0.3;
		if(strafeSpeedLeft>0) {
			strafeSpeedActual=-(((Math.pow(strafeSpeedLeft,2))/0.3)+minMotorSpeed);
		} else if (strafeSpeedRight>0){
			strafeSpeedActual=(Math.pow(strafeSpeedRight,2)/0.3)+minMotorSpeed;
		} else {
			strafeSpeedActual = 0.0;
		}

		//keep turning until within the tolerance from desired angle
		mecDrive.driveCartesian(strafeSpeedActual, desired_speed, 0.0, 0.0);
	}

	// method to turn robot to different angles automatically @aldenis @marlahna
	public void checkAutoTurn(){

		if(driverStick.getPOV()==XboxButtons.POV_LEFT){
			autoDriveFlag = true;
			autoTurn(270);		//aiming to the RIGHT
			this.lastUsedAngle = 270;

		}
		else if(driverStick.getPOV()==XboxButtons.POV_DOWN){
			autoDriveFlag = true;
			autoTurn(180);	//heading back towards driverStation
			this.lastUsedAngle = 180;

		}
		else if(driverStick.getPOV()==XboxButtons.POV_RIGHT){
			autoDriveFlag = true;
			autoTurn(90);		//aiming to the RIGHT
			this.lastUsedAngle = 90;

		}
		else if(driverStick.getPOV()==XboxButtons.POV_UP){
			autoDriveFlag = true;
			autoTurn(0);	//heading away from driverStation
			this.lastUsedAngle = 0;
		}
		else{
			autoDriveFlag=false;
		}
	}
	
	
	public double autoTurnSpeed (double futureAngle){
		//changeDrivingToPercent();  //do this manually with each "set" command
		double actualangle = this.getCurrentAngle();
		double diff = futureAngle - actualangle;	//positive deg for right turns
		// make the diff-values range from  -179 to 179
		if (diff > 180){ 			//to handle extra large right turns
			diff = diff - 360;
		} else if (diff < -180){	//to handle extra large left turns
			diff = diff + 360;
		}
		SmartDashboard.putNumber("AutoTurn Diff", diff);

		double angle_tolerance = 5.0;
		double min_speed = 0.4;
		double desired_speed;

		//adjust speeds to decrease as you approach the desired angle
		desired_speed = (1.0-min_speed) * (Math.abs(diff)/180) + min_speed;

		// assigning speed based on positive or negative - Kahlil & Malachi P
		if(diff > angle_tolerance ){  //right hand turn - speed
			desired_speed = -desired_speed;
		} else if(diff < -angle_tolerance){ // left hand turn +speed
			desired_speed = desired_speed;		
		} else{
			desired_speed = 0.0;
		}

		SmartDashboard.putNumber("AutoTurn Speed", desired_speed);
		System.out.println("ANGLE: "+ actualangle + " DIFF IS: " + diff + " DESIRED SPEED IS " + desired_speed);
	
		return desired_speed;	
	}

	//method to update the angle the robot is facing on the field -Aldenis G
	public int getCurrentAngle(){

		int rawAngle = Math.floorMod((int)navx.getAngle(), 360);
		fieldAngleDifference = Math.floorMod((int)fieldAngleDifference, 360);
		int fieldAngle = Math.floorMod((rawAngle -(int) fieldAngleDifference), 360);

		SmartDashboard.putNumber("rawAngle", rawAngle);
		SmartDashboard.putNumber("fieldAngle", fieldAngle);
		System.out.println("rawAngle:"+rawAngle);
		System.out.println("fieldAngle:"+fieldAngle);

		return fieldAngle;
	}

	

}
