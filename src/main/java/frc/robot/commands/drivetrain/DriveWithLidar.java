/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
package frc.robot.commands.drivetrain;

import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class DriveWithLidar extends MoveHeading {
    //private final Logger LOG = LogManager.getLogger(DriveWithLidar.class);
    private double inchThreshold;
    private double startingDistance;

    public DriveWithLidar() {
        super(0, 0);

        targetHeading = Robot.driveTrain.getPigeonYaw();
        moveThresholdTicks = 500;
        inchThreshold = 0.5;
        angleThresholdDeg = 2;
        onTargetMinCount = 10;
        //LOG.info(getName() + " Constructed");
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();
        //LOG.info(getName() + " Initialized");
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double adjustedDistance = Robot.driveTrain.getLidar().getDistance() - startingDistance;
        right.set(ControlMode.Position, adjustedDistance, DemandType.AuxPID, targetHeading);
        left.follow(right, FollowerType.AuxOutput1);
        //LOG.info(getName() + " Executed");
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (initialCheckCount < checkThreshold) {
            initialCheckCount++;
            return false;
        }

        double distError = Math.abs(Robot.driveTrain.getLidar().getDistance() - (startingDistance - distance));

        boolean isFinished = false;

        boolean isOnTarget = (Math.abs(distError) < inchThreshold);

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
        //LOG.info(getName() + " Ended");
    }

    @Override
    public void setRequirements(Set<Subsystem> requirements) {
        super.setRequirements(requirements);
    }
}
*/