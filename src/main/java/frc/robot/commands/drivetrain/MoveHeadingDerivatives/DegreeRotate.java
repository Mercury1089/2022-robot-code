/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.MoveHeadingDerivatives;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveTrainSide;
import frc.robot.util.MercMath;

public class DegreeRotate extends MoveHeading {

    private DriveTrain driveTrain;
    private double angleError;
    private int angleThresholdDeg;

    public DegreeRotate(double angleToTurn, DriveTrain driveTrain) {
        super(0, angleToTurn, driveTrain);

        this.setName("DegreeRotate");
        this.driveTrain = driveTrain;

        moveThresholdTicks = 100;
        angleThresholdDeg = 1;
        onTargetMinCount = 3;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();
        this.driveTrain.configPIDSlots(DriveTrainSide.RIGHT, DriveTrain.DRIVE_PID_SLOT, DriveTrain.DRIVE_SMOOTH_TURN_SLOT);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        super.execute();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        angleError = right.getClosedLoopError(DriveTrain.DRIVE_SMOOTH_TURN_SLOT);

        angleError = MercMath.pigeonUnitsToDegrees(angleError);

        boolean isFinished = false;

        boolean isOnTarget = (Math.abs(angleError) < angleThresholdDeg);

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

        String sdPrefix = driveTrain.getName() + "/" + getName();
        SmartDashboard.putNumber(sdPrefix + "/onTargetCount", onTargetCount);
        SmartDashboard.putNumber(sdPrefix + "/onTargetMinCount", onTargetMinCount);
        SmartDashboard.putNumber(sdPrefix + "/angleError", angleError);
        SmartDashboard.putBoolean(sdPrefix + "/isOnTarget", isOnTarget);
        

        return isFinished;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
