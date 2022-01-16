/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.MoveHeadingDerivatives;

import frc.robot.subsystems.DriveTrain;

public class DegreeRotate extends MoveHeading {

    private DriveTrain driveTrain;

    public DegreeRotate(double angleToTurn, DriveTrain driveTrain) {
        super(0, angleToTurn, driveTrain);

        this.setName("DegreeRotate");
        this.driveTrain = driveTrain;
        onTargetMinCount = 3;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();
        this.driveTrain.configPIDSlots(DriveTrain.DRIVE_PID_SLOT, DriveTrain.DRIVE_SMOOTH_TURN_SLOT);
    }

}
