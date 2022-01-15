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
        moveThresholdTicks = 500;
        onTargetMinCount = 10;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        super.execute();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (initialCheckCount < checkThreshold) {
            initialCheckCount++;
            return false;
        }

        double distError = driveTrain.getDistanceError();

        boolean isFinished = false;

        boolean isOnTarget = (Math.abs(distError) < moveThresholdTicks);

        if (isOnTarget) {
            onTargetCount++;
        } else {
            if (onTargetCount > 0)
                onTargetCount = 0;
        }

        if (onTargetCount > onTargetMinCount) {
            isFinished = true;
            onTargetCount = 0;
        }

        return isFinished;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
