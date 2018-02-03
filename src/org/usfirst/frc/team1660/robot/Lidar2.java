/* Class to use Garmin Lidar_Lite_v3 sensor
 * Code based off of Carlos (FRC Team 103)
 * https://www.chiefdelphi.com/forums/showthread.php?threadid=160611
 */

package org.usfirst.frc.team1660.robot;


import java.nio.ByteBuffer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lidar2 {

	private I2C i2c;
	private java.util.Timer updater;
	private ByteBuffer buffer = ByteBuffer.allocateDirect(3);
	private volatile int distance;
	private int measurementCount = 0;

	private static final int LIDAR_BUSY_MASK = 0x01;
	private static final int LIDAR_COMMAND_ACQUIRE_WITHOUT_CORRECTION = 0x03;
	private static final int LIDAR_COMMAND_ACQUIRE_WITH_CORRECTION = 0x04;
	private static final int LIDAR_CONFIG_REGISTER = 0x00;
	private static final int LIDAR_STATUS_REGISTER = 0x01;
	private static final int LIDAR_SIG_COUNT = 0x02;
	private static final int LIDAR_ACQ_CONFIG = 0x04;
	private static final int LIDAR_THRESHOLD_BYPASS = 0x1c;
	private static final int LIDAR_DISTANCE_REGISTER = 0x8f;
	
	private static final int LIDAR_ADDR = 0x62;

	private static final int UPDATE_PERIOD = 20; // in milliseconds
	private static final int RETRY_COUNT = 50;

	Port port = I2C.Port.kMXP;
	//Port port = I2C.Port.kOnboard;

	
	public Lidar2() {

		i2c = new I2C(port, LIDAR_ADDR);

		updater = new java.util.Timer();
		updater.schedule(new TimerTask() {
			@Override
			public void run() {
				distance = getUpdatedDistance();
			}
		}, 0, UPDATE_PERIOD);
		System.out.println("Started");
	}

	// Distance in cm
	public int getDistance_cm() {
		SmartDashboard.putNumber("LidarDistance_cm",distance);
		return distance;
	}
	
	// Distance in inches
	public double getDistance_inches() {
		double inches = distance * 2.54;
		SmartDashboard.putNumber("LidarDistance_in",inches);
		return inches;
	}

	// Update distance variable
	public int getUpdatedDistance() {
		i2c.write(LIDAR_CONFIG_REGISTER, LIDAR_ACQ_CONFIG); // Initiate measurement

		int busy;
		int loopnum = 0;
		do {
			if (loopnum > 1000) {
				i2c.write(0x00, 0x00);
				sleep(1000);
				i2c.write(LIDAR_CONFIG_REGISTER, LIDAR_ACQ_CONFIG);
				sleep(1);
				loopnum = 0;
			}
			sleep(20);
			busy = readByte(LIDAR_STATUS_REGISTER);
			loopnum ++;
		} while ((busy & 0x01) != 0x00);
		int val = readShort(LIDAR_DISTANCE_REGISTER);
		sleep(10);
		System.out.println("Lidar distance: "+val);
		return distance;
	}

	public void sleep(long millis) {
		try { Thread.sleep(millis); } catch (InterruptedException e) { e.printStackTrace(); }
	}

	public int readByte(int register) {
		buffer.put(0, (byte) register);
		i2c.writeBulk(buffer, 1);
		i2c.readOnly(buffer, 1);
		return buffer.get(0) & 0xFF;
	}

	public int readShort(int register) {
		buffer.put(0, (byte) register);
		i2c.writeBulk(buffer, 1);
		i2c.readOnly(buffer, 2);
		return buffer.getShort(0) & 0xFFFF;
	}
}
