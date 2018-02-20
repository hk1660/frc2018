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
import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
	PowerDistributionPanel pdp;

	//PID coefficient constants
	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;
	static final double kToleranceDegrees = 2.0;

	//angle variables
	double offsetAngle = 0.0;
	double roboAngle = 0.0;
	double lastUsedAngle;
	boolean autoTurnFlag = false;	//automatic driving
	boolean fieldDrivingFlag = false; //is using field-oriented driving
	double rotateToAngleRate;

	//drive parameters
	double strafeParameter = 0.0;
	double forwardParameter = 0.0;
	double turnParameter = 0.0;
	double angleParameter = 0.0;


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
		
		//PDP init
		pdp = new PowerDistributionPanel(RobotMap.PDP_ID);

	}


	public void drive() {
		mecDrive.driveCartesian(strafeParameter, forwardParameter, turnParameter, angleParameter);
	}
	
	//method to drive straight in AUTO
	public void autoDriveStraight(double fwdSpeed){
		strafeParameter = 0.0;
		forwardParameter = fwdSpeed;
		turnParameter = 0.0;
		angleParameter = navx.getAngle();
		
		drive();
	}

	//method to check joysticks for driving the robot -Nana B & Matthew W
	public void checkDriving() {

		double strafeJoy = squareIt(driverStick.getRawAxis(RobotMap.STRAFE_AXIS)) ; // right and left on the left thumb stick?
		double forwardJoy = -squareIt(driverStick.getRawAxis(RobotMap.FORWARD_AXIS));// up and down on left thumb stick?
		double turnJoy = squareIt(driverStick.getRawAxis(RobotMap.TURN_AXIS));// right and left on right thumb stick

		if(autoTurnFlag == false && fieldDrivingFlag){

			strafeParameter = strafeJoy;
			forwardParameter = forwardJoy;
			turnParameter = turnJoy;
			angleParameter = navx.getAngle();
		}	
		else if (autoTurnFlag == false && fieldDrivingFlag == false){
			strafeParameter = strafeJoy;
			forwardParameter = forwardJoy;
			turnParameter = turnJoy;
			angleParameter = 0.0;
		}
		else if (autoTurnFlag == true) {
			strafeParameter = strafeJoy;
			forwardParameter = forwardJoy;
			turnParameter = this.rotateToAngleRate;
			angleParameter = navx.getAngle();
		}

		drive();

		//Prints
		SmartDashboard.putNumber("forwardParam",	forwardParameter);
		SmartDashboard.putNumber("strafeParam",	strafeParameter);
		SmartDashboard.putNumber("turnParam",	turnParameter);
		SmartDashboard.putNumber("angleParam",	angleParameter);
		SmartDashboard.putBoolean("AutoTurning?", autoTurnFlag);

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


	// method to turn robot to different angles automatically @aldenis @marlahna
	public void checkAutoTurn(){

		if(driverStick.getPOV()==RobotMap.FACE_LEFT_POV){
			autoTurn(RobotMap.LEFT_WALL_ANGLE);						//aiming to the LEFT
		}
		else if(driverStick.getPOV()==RobotMap.FACE_BACKWARD_POV){
			autoTurn(RobotMap.BACK_WALL_ANGLE);						//heading back towards driverStation
		}
		else if(driverStick.getPOV()==RobotMap.FACE_RIGHT_POV){
			autoTurn(RobotMap.RIGHT_WALL_ANGLE);					//aiming to the RIGHT
		}
		else if(driverStick.getPOV()==RobotMap.FACE_FORWARD_POV){
			autoTurn(RobotMap.FRONT_WALL_ANGLE);					//heading away from driverStation
		}
		else{
			autoTurnFlag=false;
		}
	}

	public void autoTurn(double futureAngle){
		autoTurnFlag = true;
		turnController.enable();
		turnController.setSetpoint(futureAngle);
		turnParameter = rotateToAngleRate;
		drive();

		SmartDashboard.putNumber("LastAutoAngle", futureAngle);
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
		double desiredTurnSpeed;

		//adjust speeds to decrease as you approach the desired angle
		desiredTurnSpeed = (max_speed-min_speed) * (Math.abs(diff)/180) + min_speed;

		// assigning speed based on positive or negative - Kahlil & Malachi P
		if(diff > angle_tolerance ){  //right hand turn - speed
			desiredTurnSpeed = -desiredTurnSpeed;
		} else if(diff < -angle_tolerance){ // left hand turn +speed
			desiredTurnSpeed = desiredTurnSpeed;		
		} else{
			desiredTurnSpeed = 0.0;
		}

		SmartDashboard.putNumber("AutoTurn Speed", desiredTurnSpeed);
		System.out.println("ANGLE: "+ actualangle + " DIFF IS: " + diff + " DESIRED SPEED IS " + desiredTurnSpeed);

		return desiredTurnSpeed;	
	}
	 */

	//method to update the angle the robot is facing on the field -Aldenis G
	public int getCurrentAngle(){

		int rawAngle = Math.floorMod((int)navx.getAngle(), 360);
		offsetAngle = Math.floorMod((int)offsetAngle, 360);
		int fieldAngle = Math.floorMod((rawAngle -(int) offsetAngle), 360);

		SmartDashboard.putNumber("rawAngle", rawAngle);
		SmartDashboard.putNumber("offsetAngle", offsetAngle);
		SmartDashboard.putNumber("fieldAngle", fieldAngle);
		//System.out.println("rawAngle:"+rawAngle);
		//System.out.println("fieldAngle:"+fieldAngle);

		return fieldAngle;
	}


	public void navxReset() {
		navx.reset();
	}

	public void goForwardPercentOutput(double speed){
		this.mecDrive.driveCartesian(0.0, speed, 0.0, navx.getAngle()); 
	}

	
	//auto method to drive at a constant voltage
	public void goForwardVoltage(double desiredVoltage){
		
		double mVoltage = pdp.getVoltage();
		double desiredPercentForwardSpeed = desiredVoltage / mVoltage;
		
		//mecDrive.driveCartesian(0.0, desiredPercentForwardSpeed, 0.0, navx.getAngle()); 
		mecDrive.driveCartesian(0.0, desiredPercentForwardSpeed, 0.0, 0.0); 
		
		
		SmartDashboard.putNumber("frontLeftCurrent", this.frontLeft.getOutputCurrent());
		SmartDashboard.putNumber("backLeftCurrent", this.backLeft.getOutputCurrent());
		SmartDashboard.putNumber("frontRightCurrent", this.frontRight.getOutputCurrent());
		SmartDashboard.putNumber("backRightCurrent", this.backRight.getOutputCurrent());
		
		
	}

	//auto method to turn at a constant voltage
	public void turnVoltage(double desiredVoltage){
		
		double mVoltage = pdp.getVoltage();
		double desiredPercentTurnSpeed = desiredVoltage / mVoltage;
		
		mecDrive.driveCartesian(0.0, 0.0, desiredPercentTurnSpeed, 0.0); 
		
		SmartDashboard.putNumber("frontLeftCurrent", this.frontLeft.getOutputCurrent());
		SmartDashboard.putNumber("backLeftCurrent", this.backLeft.getOutputCurrent());
		SmartDashboard.putNumber("frontRightCurrent", this.frontRight.getOutputCurrent());
		SmartDashboard.putNumber("backRightCurrent", this.backRight.getOutputCurrent());
		
		
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
