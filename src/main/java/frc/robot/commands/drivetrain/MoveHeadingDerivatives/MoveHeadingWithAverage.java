/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.MoveHeadingDerivatives;

import frc.robot.subsystems.DriveTrain;

public class MoveHeadingWithAverage extends MoveHeading {

    public MoveHeadingWithAverage(double distance, double heading, DriveTrain driveTrain) {
        super(distance, heading, driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        onTargetCount = 0;
        initialCheckCount = 0;

        this.driveTrain.configPIDSlots(DriveTrain.DRIVE_PID_SLOT, DriveTrain.DRIVE_SMOOTH_MOTION_SLOT);
        targetDistance = distance + this.driveTrain.getPositionInInches();
        targetHeading = heading + this.driveTrain.getPigeonYawInDegrees();

        driveTrain.moveHeading(targetDistance, targetHeading);
    }
}
