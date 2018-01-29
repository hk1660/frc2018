package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;;

public class TeleopPIDInput implements PIDSource{
//TODO: change class name to rotation
	HKDrive m_hkdrive;
	public TeleopPIDInput(HKDrive hkdrive) {
		m_hkdrive = hkdrive;
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		return;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		return m_hkdrive.getCurrentAngle();
	}

}
