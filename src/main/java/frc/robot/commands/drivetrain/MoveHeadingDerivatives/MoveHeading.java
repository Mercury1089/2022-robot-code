/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.MoveHeadingDerivatives;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.MercMath;

public class MoveHeading extends CommandBase {
    protected final int CLOSED_LOOP_TIME_MS = 1;
    protected int onTargetMinCount;
    protected int checkThreshold = 50;

    protected double distance, targetHeading;
    protected int onTargetCount, initialCheckCount, resetCounter;

    protected boolean reset;

    protected DriveTrain driveTrain;

    /**
     * Move with heading assist from pigeon
     *
     * @param distance distance to move in inches
     * @param heading  heading to turn to for the pigeon
     */
    public MoveHeading(double distance, double heading, DriveTrain driveTrain) {
        super.addRequirements(driveTrain);
        setName("MoveHeading");
        this.driveTrain = driveTrain;

        onTargetMinCount = 4; // 100 millis

        this.distance = distance;
        this.targetHeading = heading;
    }
    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        this.driveTrain.stop();
        this.driveTrain.resetEncoders();

        onTargetCount = 0;
        initialCheckCount = 0;

        this.driveTrain.configPIDSlots(DriveTrain.DRIVE_PID_SLOT, DriveTrain.DRIVE_SMOOTH_MOTION_SLOT);

        this.driveTrain.resetPigeonYaw();

        reset = driveTrain.getPigeonYaw() == 0.0 && 
                driveTrain.getLeftEncPositionInTicks() == 0.0 && 
                driveTrain.getRightEncPositionInTicks() == 0.0;
        resetCounter = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        /* Configured for MotionMagic on Quad Encoders and Auxiliary PID on Pigeon */
        if(!reset){
            if (resetCounter % 5 == 0) {
                driveTrain.resetEncoders();
                driveTrain.resetPigeonYaw();
            }
            resetCounter++;
            reset = driveTrain.getPigeonYaw() == 0.0 && 
                    driveTrain.getLeftEncPositionInTicks() == 0.0 && 
                    driveTrain.getRightEncPositionInTicks() == 0.0;
        } else {
            driveTrain.moveHeading(distance, targetHeading);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (initialCheckCount < checkThreshold) {
            initialCheckCount++;
            return false;
        }
        boolean isFinished = false;

        if (driveTrain.isOnTarget()) {
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
        this.driveTrain.stop();
        this.driveTrain.configVoltage(DriveTrain.NOMINAL_OUT, DriveTrain.PEAK_OUT);
    }
}
