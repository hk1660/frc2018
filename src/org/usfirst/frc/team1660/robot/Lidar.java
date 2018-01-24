package org.usfirst.frc.team1660.robot;

/**
 *
 */

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.I2C;

public class Lidar {

	I2C m_i2c;

	public void initLidar(){	
		m_i2c = new I2C(I2C.Port.kMXP,0x62);
	}

	public int getDistance() {

		byte[] buffer; 
		buffer = new byte[2];

		m_i2c.write(0x00, 0x04);		//what is this mean?
		Timer.delay(0.04);
		m_i2c.read(0x8f, 2, buffer);
		
		int distance=(int)Integer.toUnsignedLong(buffer[0] << 8) + Byte.toUnsignedInt(buffer[1])
		
		SmartDashboard.putNumber("laser", distance);
		System.out.println("laser:"+distance);

		return distance;	
	}


}