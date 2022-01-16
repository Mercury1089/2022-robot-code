/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.MoveHeadingDerivatives;

import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends MoveHeading{

    /**
     * Construct Drive Distance w / Motion Magic
     *
     * @param distance in inches
     */
    public DriveDistance(double distance, DriveTrain driveTrain) {
        super(distance, 0, driveTrain);
        setName("DriveDistance");
        onTargetMinCount = 10;
    }

}
