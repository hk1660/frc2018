package org.usfirst.frc.team1660.robot;

/**
 *
 */


import java.util.TimerTask;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.I2C;

public class Lidar extends SensorBase {

	private I2C i2c;
	private byte[] buffer;
	private final int LIDAR_ADDR = 0x62;
	private final int LIDAR_CONFIG_REGISTER = 0x00;
	private final int LIDAR_DISTANCE_REGISTER = 0x8f;
	private java.util.Timer updater;
	

	public void initLidar(){	
		i2c = new I2C(I2C.Port.kMXP,LIDAR_ADDR);
		buffer = new byte[2];
		updater = new java.util.Timer();

	}

	public int getDistance() {

		i2c.write(LIDAR_CONFIG_REGISTER, 0x04);		//what is this mean?
		Timer.delay(0.04);
		i2c.read(LIDAR_DISTANCE_REGISTER, 2, buffer);

		int distance=(int)Integer.toUnsignedLong(buffer[0] << 8) + Byte.toUnsignedInt(buffer[1]);

		SmartDashboard.putNumber("laser", distance);
		System.out.println("laser:"+distance);

		return distance;	
	}

    /**
     * Return Distance in Inches
     * 
     * @return distance in inches
     */
    public double getDistanceIn() {                                     
        return (double) getDistance() * 0.393701;
    }
	

    /**
     * Start 10Hz polling of LIDAR sensor, in a background task. Only allow 10 Hz. polling at the
     * moment.
     */
    public void start() {
        updater.scheduleAtFixedRate(new LIDARUpdater(), 0, 100);
    }


    /**
     * Stop the background sensor-polling task.
     */
    public void stop() {
        updater.cancel();
        updater = new java.util.Timer();
    }


    /**
     * Read from the sensor and update the internal "distance" variable with the result.
     */
    private void update() {
        i2c.write(LIDAR_CONFIG_REGISTER, 0x04); // Initiate measurement
        Timer.delay(0.04); // Delay for measurement to be taken
        i2c.read(LIDAR_DISTANCE_REGISTER, 2, buffer); // Read in measurement
        Timer.delay(0.005); // Delay to prevent over polling
    }

    /**
     * Timer task to keep distance updated
     *
     */
    private class LIDARUpdater extends TimerTask {
        public void run() {
            while (true) {
                update();
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    
    
    
	@Override
	public void initSendable(SendableBuilder builder) {
		// TODO Auto-generated method stub

	}


}