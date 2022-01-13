/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Add your docs here.
 */
public class Ultrasonic {
    private final double SCALING_FACTOR = 5.0;
    AnalogInput ultrasonic;

    public Ultrasonic(int port) {
        ultrasonic = new AnalogInput(port);
    }

    public double getDistance() {
        return ultrasonic.getVoltage() * SCALING_FACTOR;
    }

    public double getRawVoltage() {
        return ultrasonic.getVoltage();
    }
}