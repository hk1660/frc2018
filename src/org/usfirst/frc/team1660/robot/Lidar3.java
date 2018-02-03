  /*Based off of FRC2102's LIDARLite class
 * https://github.com/Paradox2102/Artemis/blob/master/src/team2102/lib/LIDARLite.java
 * 
 * See implementation in the Navigation class:
 * https://github.com/Paradox2102/Artemis/blob/master/src/team2102/robot/Navigator.java
 */

package org.usfirst.frc.team1660.robot;

import java.nio.ByteBuffer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.hal.I2CJNI;


public class Lidar3 implements PIDSource {

		private static final byte k_deviceAddress = 0x62;

		private final byte m_port;

		private final ByteBuffer m_buffer = ByteBuffer.allocateDirect(2);

		public Lidar3() {
			Port port = I2C.Port.kOnboard;
			m_port = (byte) port.value;
			I2CJNI.i2CInitialize(m_port);
		}

		public void startMeasuring() {
			writeRegister(0x04, 0x08 | 32); // default plus bit 5
			writeRegister(0x11, 0xff);
			writeRegister(0x00, 0x04);
		}

		public void stopMeasuring() {
			writeRegister(0x11, 0x00);
		}

		public int getDistance() {
			return readShort(0x8f);
		}

		private int writeRegister(int address, int value) {
			m_buffer.put(0, (byte) address);
			m_buffer.put(1, (byte) value);

			return I2CJNI.i2CWrite(m_port, k_deviceAddress, m_buffer, (byte) 2);
		}

		private short readShort(int address) {
			m_buffer.put(0, (byte) address);
			I2CJNI.i2CWrite(m_port, k_deviceAddress, m_buffer, (byte) 1);
			I2CJNI.i2CRead(m_port, k_deviceAddress, m_buffer, (byte) 2);
			return m_buffer.getShort(0);
		}

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			if (pidSource != PIDSourceType.kDisplacement) {
				throw new IllegalArgumentException("Only displacement is supported");
			}
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			return getDistance();
		}
	};
