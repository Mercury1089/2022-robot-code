// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors.REVColorMux;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/** Add your docs here. */

// Based off: https://github.com/FRC3005/I2CMux-Example/blob/main/src/main/java/frc/robot/TCA9548A.java
public class TCA9548A {
    private final int I2CADDRESS;
    private final I2C i2c;

    public TCA9548A() {
        I2CADDRESS = 0x70; // Default address for the common PCB available
        i2c = new I2C(Port.kMXP, I2CADDRESS);
    }

    /**
   * Read list of enabled buses from the device.
   *
   * @return bit field of enabled buses
   */
  public int enabledBuses() {
    byte[] result = new byte[1];
    i2c.readOnly(result, 1);
    return result[0];
  }

  /**
   * Set the list of enabled buses
   *
   * @param buses list of buses to enable
   */
  public void setEnabledBuses(int... buses) {
    int writeValue = 0;
    for (int b : buses) {
      if (b >= availableBuses() || b < 0) {
        DriverStation.reportError("Invalid bus enabled on I2C Mux: " + b, true);
      } else {
        writeValue |= 1 << b;
      }
    }
    i2c.write(I2CADDRESS, writeValue);
  }

  /**
   * Number of available buses
   *
   * @return number of available buses
   */  
  public int availableBuses() {
    return 8;
  }


}
