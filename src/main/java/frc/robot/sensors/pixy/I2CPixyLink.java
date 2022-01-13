package frc.robot.sensors.pixy;

import edu.wpi.first.wpilibj.I2C;

/**
 * PixyCam implementation using the I2C interface
 * Thanks to croadfeldt (https://github.com/croadfeldt/wpilib_pixy_spi_java) for the original implementation
 */
public class I2CPixyLink implements IPixyLink {
    private static final int PIXY_I2C_ADDRESS = 0x54;
    private final I2C i2c;

    public I2CPixyLink() {
        this(I2C.Port.kOnboard);
    }

    public I2CPixyLink(I2C.Port port) {
        i2c = new I2C(port, PIXY_I2C_ADDRESS);
    }

    /**
     * Converts bytes from the PIXY into readable data
     *
     * @return int data from PIXY
     */
    public int getWord() {
        byte[] rawData = new byte[2];

        try {
            i2c.readOnly(rawData, 2);
        } catch (RuntimeException e) {
            return 0;
        }

        if (rawData.length < 2) {
            return 0;
        }
        return (((int) rawData[1] & 0xff) << 8) | ((int) rawData[0] & 0xff);
    }
}

