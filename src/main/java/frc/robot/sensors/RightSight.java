/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
//import org.usfirst.frc1089.lib.Ports;

/**
 * Add your docs here.
 */
public class RightSight {

    private final int DIO_PORT;
    private DigitalInput RightSight;

    /**
     * Sets the sensor to its respective port on the DIO
     */
    public RightSight(int dioPort) {
        DIO_PORT = dioPort;
        RightSight = new DigitalInput(DIO_PORT);
    }

    /**
     * checks the DIO port assigned to the sensor for its alignment
     */
    public boolean getAlignment() {
        return RightSight.get();
    }
}