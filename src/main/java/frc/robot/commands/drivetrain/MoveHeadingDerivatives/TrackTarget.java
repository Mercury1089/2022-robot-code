/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.MoveHeadingDerivatives;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightCamera;

public class TrackTarget extends MoveHeading {
    private double allowableDistError = 19; //inches

    private DriveTrain driveTrain;
    private LimelightCamera limelightCamera;

    public TrackTarget(DriveTrain driveTrain, LimelightCamera limelightCamera) {
        super(0, 0, driveTrain);

        setName("TrackTarget");
        this.driveTrain = driveTrain;
        this.limelightCamera = limelightCamera;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();
        this.driveTrain.configPIDSlots(DriveTrain.DRIVE_PID_SLOT, DriveTrain.DRIVE_SMOOTH_MOTION_SLOT);
        this.driveTrain.configClosedLoopPeakOutput(DriveTrain.DRIVE_PID_SLOT, .3);
        this.driveTrain.configClosedLoopPeakOutput(DriveTrain.DRIVE_SMOOTH_MOTION_SLOT, .25);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double adjustedDistance = this.limelightCamera.getLimelight().getRawVertDistance() - allowableDistError;
        //adjustedDistance *= Robot.driveTrain.getDirection().dir;
        double adjustedHeading = this.limelightCamera.getLimelight().getTargetCenterXAngle();
        driveTrain.moveHeading(adjustedDistance, adjustedHeading);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (initialCheckCount < checkThreshold) {
            initialCheckCount++;
            return false;
        }

        boolean isFinished = false;

        boolean isOnTarget = driveTrain.isOnTarget() &&
                this.limelightCamera.getLimelight().isSafeToTrack();

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
        this.driveTrain.configClosedLoopPeakOutput(DriveTrain.DRIVE_PID_SLOT, .75);
        this.driveTrain.configClosedLoopPeakOutput(DriveTrain.DRIVE_SMOOTH_MOTION_SLOT, 1.0);
    }
}
