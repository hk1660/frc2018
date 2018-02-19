package org.usfirst.frc.team1660.robot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Wraps an analog input for a Rev Robotics Analog Pressure sensor.
 * 
 * Adapted from: https://github.com/Team254/FRC-2016-Public/blob/master/src/com/team254/lib/util/RevRoboticsAirPressureSensor.java
 *
 * Spec Sheet: http://www.revrobotics.com/content/docs/REV-11-1107-DS.pdf
 */
 //to read air pressure
public class AirPressureSensor {

    private final AnalogInput m_pressureSensor;

    public AirPressureSensor(int analogInputNumber) {
        m_pressureSensor = new AnalogInput(analogInputNumber);
    }

    public double getAirPressurePsi() {
        // taken from the spec sheet
        return ((250.0 * m_pressureSensor.getVoltage()) / 5.0) - 25.0;
    }
    
    public void updateAirPressureDisplay() {
    	SmartDashboard.putNumber("Air Pressure: ", getAirPressurePsi());
    }
}
