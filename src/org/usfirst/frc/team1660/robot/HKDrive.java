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
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class HKDrive implements PIDOutput {

	/*----DECLARED GLOBAL VARIABLES-------*/

	//Drivetrain Declarations
	private WPI_TalonSRX frontLeft;
	private WPI_TalonSRX backLeft;
	private WPI_TalonSRX frontRight;
	private WPI_TalonSRX backRight;
	private MecanumDrive mecDrive;
	private AHRS navx;
	PIDController turnController;

	//PID coefficient constants
	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;
	static final double kToleranceDegrees = 2.0f;

	//angle variables
	double offsetAngle = 0.0;
	double roboAngle = 0.0;
	double lastUsedAngle;
	boolean autoDriveFlag = false;	//automatic driving
	boolean fieldDrivingFlag = false; //is using field-oriented driving
	double rotateToAngleRate;

	
	//Joystick fields
	private Joystick driverStick;

	public HKDrive(Joystick joy){
		this.driverStick = joy;

	}

	//HKDrive init method
	public void driveInit() {

		//Drivetrain Initializations
		frontLeft = new WPI_TalonSRX(RobotMap.DRIVE_FRONT_LEFT_CHANNEL);
		backLeft = new WPI_TalonSRX(RobotMap.DRIVE_BACK_LEFT_CHANNEL);
		frontRight = new WPI_TalonSRX(RobotMap.DRIVE_FRONT_RIGHT_CHANNEL);
		backRight = new WPI_TalonSRX(RobotMap.DRIVE_BACK_RIGHT_CHANNEL);

		//we think the constructor switched the 3rd & 4th parameters
		mecDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

		//navx intialization
		try {
			navx = new AHRS(SPI.Port.kMXP); //navX-MXP initialized with (SPI, I2C, TTL UART) and USB //http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

		//PID control initialization
		turnController = new PIDController(kP, kI, kD, kF, navx, this);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);

		/* Add the PID Controller to the Test-mode dashboard, allowing manual  */
		/* tuning of the Turn Controller's P, I and D coefficients.            */
		/* Typically, only the P value needs to be modified.                   */
		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);

	}


	//method to check joysticks for driving the robot -Nana B & Matthew W
	public void checkDriving() {

		double strafe = squareIt(driverStick.getRawAxis(RobotMap.STRAFE_AXIS)) ; // right and left on the left thumb stick?
		double forward = squareIt(driverStick.getRawAxis(RobotMap.FORWARD_AXIS));// up and down on left thumb stick?
		double turn = squareIt(driverStick.getRawAxis(RobotMap.TURN_AXIS));// right and left on right thumb stick


		//MECANUM -Malachi P
		if(autoDriveFlag == false ){

			//we think the parameters were wrong: S>T>F not F>S>T, actually... S>F>T

			if(fieldDrivingFlag) {
				mecDrive.driveCartesian(strafe, -forward, turn, navx.getAngle());
			}	else {
				mecDrive.driveCartesian(strafe, -forward, turn, 0);
			}


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

	public void checkFieldFlag() {
		if(driverStick.getRawButton(RobotMap.FIELD_DRIVING_FLAG_ON_BUTTON)) {
			fieldDrivingFlag = true;
		}
		if(driverStick.getRawButton(RobotMap.FIELD_DRIVING_FLAG_OFF_BUTTON)) {
			fieldDrivingFlag = false;	
		}
		SmartDashboard.putBoolean("FieldDriveFlag?", fieldDrivingFlag);
	}

	//joystick method to manually resets the robot to the field's orientation -Aldenis G
	public void checkSetOffsetAngle(){

		if (driverStick.getRawButton(RobotMap.RESET_OFFSET_ANGLE_BUTTON)) {
			setOffsetAngle();
		}
	}

	public void setOffsetAngle() {
		offsetAngle = navx.getAngle();
	}

	public void autoTurn(int futureAngle){
		turnController.enable();
		turnController.setSetpoint(futureAngle);
		double desired_speed = rotateToAngleRate;		//autoTurnSpeed(futureAngle);
		mecDrive.driveCartesian(0.0, desired_speed, 0.0, 0.0);
	}

	// method to turn robot to different angles automatically @aldenis @marlahna
	public void checkAutoTurn(){

		if(driverStick.getPOV()==RobotMap.FACE_LEFT_POV){
			autoDriveFlag = true;
			autoTurn(270);		//aiming to the RIGHT
			this.lastUsedAngle = 270;

		}
		else if(driverStick.getPOV()==RobotMap.FACE_BACKWARD_POV){
			autoDriveFlag = true;
			autoTurn(180);	//heading back towards driverStation
			this.lastUsedAngle = 180;

		}
		else if(driverStick.getPOV()==RobotMap.FACE_RIGHT_POV){
			autoDriveFlag = true;
			autoTurn(90);		//aiming to the RIGHT
			this.lastUsedAngle = 90;

		}
		else if(driverStick.getPOV()==RobotMap.FACE_FORWARD_POV){
			autoDriveFlag = true;
			autoTurn(0);	//heading away from driverStation
			this.lastUsedAngle = 0;
		}
		else{
			autoDriveFlag=false;
		}
	}

/*
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
		double max_speed = 1.0;
		double desired_speed;

		//adjust speeds to decrease as you approach the desired angle
		desired_speed = (max_speed-min_speed) * (Math.abs(diff)/180) + min_speed;

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
	*/

	//method to update the angle the robot is facing on the field -Aldenis G
	public int getCurrentAngle(){

		int rawAngle = Math.floorMod((int)navx.getAngle(), 360);
		offsetAngle = Math.floorMod((int)offsetAngle, 360);
		int fieldAngle = Math.floorMod((rawAngle -(int) offsetAngle), 360);

		SmartDashboard.putNumber("rawAngle", rawAngle);
		SmartDashboard.putNumber("fieldAngle", fieldAngle);
		System.out.println("rawAngle:"+rawAngle);
		System.out.println("fieldAngle:"+fieldAngle);

		return fieldAngle;
	}


	public void navxReset() {
		navx.reset();
	}
	
	public void goForwardPercentOutput(double speed){
		this.mecDrive.driveCartesian(0.0, speed, 0.0);
	}

	public void stop(){
		this.mecDrive.driveCartesian(0.0, 0.0, 0.0);
	}

	@Override
	/* This function is invoked periodically by the PID Controller, */
	/* based upon navX-MXP yaw angle input and PID Coefficients.    */
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}



}
