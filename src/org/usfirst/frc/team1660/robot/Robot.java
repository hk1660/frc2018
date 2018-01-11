/*
 * Code for the Harlem Knights (FRC 1660) Robot for 2018
 */

package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */

public class Robot extends IterativeRobot {
	private static final int kFrontLeftChannel = 2;
	private static final int kBackLeftChannel = 3;
	private static final int kFrontRightChannel = 1;
	private static final int kBackRightChannel = 0;

	private static final int kJoystickChannel = 0;

	private MecanumDrive m_robotDrive;
	private Joystick m_stick;

	@Override
	public void robotInit() {
		Spark frontLeft = new Spark(kFrontLeftChannel);
		Spark backLeft = new Spark(kBackLeftChannel);
		Spark frontRight = new Spark(kFrontRightChannel);
		Spark backRight = new Spark(kBackRightChannel);

		// Invert the left side motors.
		// You may need to change or remove this to match your robot.
		frontLeft.setInverted(true);
		backLeft.setInverted(true);

		m_robotDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

		m_stick = new Joystick(kJoystickChannel);
	}

	@Override
	public void teleopPeriodic() {
		// Use the joystick X axis for lateral movement, Y axis for forward
		// movement, and Z axis for rotation.
		m_robotDrive.driveCartesian(m_stick.getX(), m_stick.getY(),
				m_stick.getZ(), 0.0);
	}
}
