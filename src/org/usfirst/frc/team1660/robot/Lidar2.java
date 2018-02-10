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

	
	private static final int LIDAR_ADDR = 0x62;	
	
	private static final int LIDAR_REG_ACQ_COMMAND = 0x00;		//W Device Command
	private static final int LIDAR_REG_STATUS = 0x01;			//R System Status
	private static final int LIDAR_REG_SIG_COUNT = 0x02;		//R-W max acquisition count, initial val 0x80
	private static final int LIDAR_REG_ACQ_CONFIG = 0x04;		//R-W acquisition mode control, initial val 0x08
	private static final int LIDAR_REG_SIGNAL_STRENGTH = 0x0E;	//READ the rec'd signal strength
	private static final int LIDAR_REG_FULL_DELAY_HIGH = 0x0F;	//READ distance measurement high byte
	private static final int LIDAR_REG_FULL_DELAY_LOW = 0x10;	//READ distance measurement low byte
	
	private static final int LIDAR_REG_ID_HIGH = 0x16;		//WRITE serial num high byte
	private static final int LIDAR_REG_ID_LOW = 0x17;		//WRITE serial num low byte 
	private static final int LIDAR_REG_I2C_ID_HIGH = 0x18;		//WRITE serial num high byte for I2C address unlock
	private static final int LIDAR_REG_I2C_ID_LOW = 0x19;		//WRITE serial num low byte for I2C address unlock
	private static final int LIDAR_REG_I2C_SEC_ADDR = 0x1A;		//R-W can write a new address after unlock		
	private static final int LIDAR_REG_I2C_CONFIG = 0x1E;		//R-W can write a new address after unlock		
	
	private static final int LIDAR_REG_THRESHOLD_BYPASS = 0x1C;	//R-W peak detection threshold bypass, initial val 0x00
	private static final int LIDAR_REG_POWER_CONTROL = 0x65; 	//R-W Power State Control, Initial Value of 0x80
	private static final int LIDAR_REG_DISTANCE = 0x8F;			
	
	private static final int LIDAR_REG_FIRMWARE = 0x4F;			//READ firmware version 	
	private static final int LIDAR_REG_HARDWARE = 0x41;			//READ hardware version
	
	
	/* Commands for Register 0x00 */
	private static final int LIDAR_COMMAND_RESET_DEFAULT = 0x00;
	private static final int LIDAR_COMMAND_ACQUIRE_WITHOUT_CORRECTION = 0x03;
	private static final int LIDAR_COMMAND_ACQUIRE_WITH_CORRECTION = 0x04;

	/* Flags for Register 0x01 */
	private static final int PROCESS_ERROR_FLAG_BIT = 6;
	private static final int HEALTH_FLAG_BIT = 5;
	private static final int SECONDARY_RETURN_FLAG_BIT = 4;
	private static final int INVALID_SIGNAL_FLAG_BIT = 3;
	private static final int SIGNAL_OVERFLOW_FLAG_BIT = 2;
	private static final int REFERENCE_OVERFLOW_FLAG_BIT = 1;
	private static final int BUSY_FLAG_BIT = 0; //0 device is ready, 1 device is busy taking measurements!
	
	private static final int LIDAR_BUSY_MASK = 0x01; //on bit 0 of register 1

	/* Control Modes for Register 0x04 
	 * 0 value enables, 1 value disables*/
	private static final int ENABLE_REF_PROCESS_BIT = 6;
	private static final int ENABLE_QUICK_TERMINATION_BIT = 3;
	private static final int USE_DEFAULT_REF_ACQ_BIT = 2;
	private static final int MODE_SELECT_PIN_1 = 1;	//01 Status Output Mode
	private static final int MODE_SELECT_PIN_2 = 0;	//11 Oscillator Output Mode
	
	private static final int UPDATE_PERIOD = 20; // in milliseconds
	private static final int RETRY_COUNT = 50;

	//Port port = I2C.Port.kMXP;
	Port port = I2C.Port.kOnboard;


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

	// Reset the Lidar settings
		public void resetLidar() {
			i2c.write(LIDAR_REG_ACQ_COMMAND, LIDAR_COMMAND_RESET_DEFAULT);
			System.out.println("Reset the Lidar settings");
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
		i2c.write(LIDAR_REG_ACQ_COMMAND, LIDAR_COMMAND_ACQUIRE_WITH_CORRECTION); // Initiate measurement

		//keep trying to get a value if busy, every 1000 tries do a reset
		byte[] buff = new byte[2];
		int lidarStatus;
		boolean isBusy;
		int loopnum = 0;
		do {
			if (loopnum > 1000) {
				printStatus();
				i2c.write(LIDAR_REG_ACQ_COMMAND, this.LIDAR_COMMAND_RESET_DEFAULT);
				sleep(1000);
				i2c.write(LIDAR_REG_ACQ_COMMAND, LIDAR_COMMAND_ACQUIRE_WITH_CORRECTION);
				sleep(1);
				loopnum = 0;
			}
			sleep(20);
			lidarStatus = readByte(LIDAR_REG_STATUS);
			isBusy = (lidarStatus & LIDAR_BUSY_MASK) == LIDAR_BUSY_MASK;
			loopnum ++;
		} while (isBusy); //checks last bit of the Status register

		int val = readShort(LIDAR_REG_DISTANCE);
		sleep(10);
		System.out.println("Lidar distance: "+val);
		return distance;

	}

	public byte mask(int bit) {
		return (byte) Math.pow(2, bit);
	}
	
	public void printStatus() {
		int lidarStatus = readByte(LIDAR_REG_STATUS);
		System.out.println("lidarStatus: " + lidarStatus);
		
		boolean isProcessError = (lidarStatus & mask(PROCESS_ERROR_FLAG_BIT)) == mask(PROCESS_ERROR_FLAG_BIT);
		System.out.println(PROCESS_ERROR_FLAG_BIT + " isProcessError: " + isProcessError);

		boolean isHealthy = (lidarStatus & mask(HEALTH_FLAG_BIT)) == mask(HEALTH_FLAG_BIT);
		System.out.println(HEALTH_FLAG_BIT + " isHealthy: " + isProcessError);

		boolean hasSecondaryReturn = (lidarStatus & mask(SECONDARY_RETURN_FLAG_BIT)) == mask(SECONDARY_RETURN_FLAG_BIT);
		System.out.println(SECONDARY_RETURN_FLAG_BIT + " hasSecondaryReturn: " + hasSecondaryReturn);

		boolean isInvalidSignal = (lidarStatus & mask(INVALID_SIGNAL_FLAG_BIT)) == mask(INVALID_SIGNAL_FLAG_BIT);
		System.out.println(INVALID_SIGNAL_FLAG_BIT + " isInvalidSignal: " + isInvalidSignal);
	
		boolean isSignalOverflow = (lidarStatus & mask(SIGNAL_OVERFLOW_FLAG_BIT)) == mask(SIGNAL_OVERFLOW_FLAG_BIT);
		System.out.println(SIGNAL_OVERFLOW_FLAG_BIT + " isSignalOverflow: " + isInvalidSignal);

		boolean isReferenceOverflow = (lidarStatus & mask(REFERENCE_OVERFLOW_FLAG_BIT)) == mask(REFERENCE_OVERFLOW_FLAG_BIT);
		System.out.println(REFERENCE_OVERFLOW_FLAG_BIT + " isReferenceOverflow: " + isReferenceOverflow);
		
		boolean isBusy = (lidarStatus & mask(BUSY_FLAG_BIT)) == mask(BUSY_FLAG_BIT);
		System.out.println(BUSY_FLAG_BIT + " isBusy: " + isBusy);

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
		i2c.writeBulk(buffer, 1);		//why only writing 1 byte if a short is 2 bytes??
		i2c.readOnly(buffer, 2);
		return buffer.getShort(0) & 0xFFFF;
	}
}
