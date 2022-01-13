/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.MoveHeadingDerivatives;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightCamera;
import frc.robot.subsystems.DriveTrain.DriveTrainSide;
import frc.robot.util.MercMath;

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
        this.driveTrain.configPIDSlots(DriveTrainSide.RIGHT, DriveTrain.DRIVE_PID_SLOT, DriveTrain.DRIVE_SMOOTH_MOTION_SLOT);
        this.driveTrain.configClosedLoopPeakOutput(DriveTrain.DRIVE_PID_SLOT, .3);
        this.driveTrain.configClosedLoopPeakOutput(DriveTrain.DRIVE_SMOOTH_MOTION_SLOT, .25);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double adjustedDistance = MercMath.feetToEncoderTicks(this.limelightCamera.getLimelight().getRawVertDistance() - allowableDistError);
        //adjustedDistance *= Robot.driveTrain.getDirection().dir;
        double adjustedHeading = MercMath.degreesToPigeonUnits(this.limelightCamera.getLimelight().getTargetCenterXAngle());
        right.set(ControlMode.Position, adjustedDistance, DemandType.AuxPID, adjustedHeading);
        left.follow(right, FollowerType.AuxOutput1);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (initialCheckCount < checkThreshold) {
            initialCheckCount++;
            return false;
        }

        double distError = MercMath.inchesToEncoderTicks(this.limelightCamera.getLimelight().getRawVertDistance() - allowableDistError),
                angleError = MercMath.degreesToPigeonUnits(this.limelightCamera.getLimelight().getTargetCenterXAngle());

        angleError = MercMath.pigeonUnitsToDegrees(angleError);
        distError *= this.driveTrain.getDirection().dir;

        String sdPrefix = driveTrain.getName() + "/" + getName();
        SmartDashboard.putNumber(sdPrefix + "angleError", angleError);
        SmartDashboard.putNumber(sdPrefix + "distError", distError);

        System.out.println(distError);

        boolean isFinished = false;

        boolean isOnTarget = (Math.abs(distError) < moveThresholdTicks &&
                Math.abs(angleError) < DriveTrain.ANGLE_THRESHOLD_DEG &&
                this.limelightCamera.getLimelight().isSafeToTrack());

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
