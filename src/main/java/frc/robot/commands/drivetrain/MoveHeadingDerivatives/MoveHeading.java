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
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveTrainSide;
import frc.robot.util.MercMath;

public class MoveHeading extends CommandBase {
    protected final int CLOSED_LOOP_TIME_MS = 1;
    protected int moveThresholdTicks;   // ticks
    protected int onTargetMinCount; // 100 millis
    protected int checkThreshold = 50;
    protected BaseMotorController left, right;

    protected double distance, targetHeading;
    protected int onTargetCount, initialCheckCount, resetCounter;

    protected boolean reset;

    private DriveTrain driveTrain;

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

        left = this.driveTrain.getLeftLeader();
        right = this.driveTrain.getRightLeader();

        moveThresholdTicks = 500;
        onTargetMinCount = 4;

        this.distance = MercMath.inchesToEncoderTicks(distance);
        this.targetHeading = MercMath.degreesToPigeonUnits(heading);
    }
    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        this.driveTrain.stop();
        this.driveTrain.resetEncoders();

        if (!this.driveTrain.isInMotionMagicMode())
            this.driveTrain.initializeMotionMagicFeedback();

        onTargetCount = 0;
        initialCheckCount = 0;

        /* Motion Magic Configurations */
        right.configMotionAcceleration(1000);
        right.configMotionCruiseVelocity((int) MercMath.revsPerMinuteToTicksPerTenth(DriveTrain.MAX_RPM));

        int closedLoopTimeMs = 1;
        right.configClosedLoopPeriod(0, closedLoopTimeMs);
        right.configClosedLoopPeriod(1, closedLoopTimeMs);

        right.configAuxPIDPolarity(false);

        this.driveTrain.configPIDSlots(DriveTrainSide.RIGHT, DriveTrain.DRIVE_PID_SLOT, DriveTrain.DRIVE_SMOOTH_MOTION_SLOT);

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
            right.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, targetHeading);
            left.follow(right, FollowerType.AuxOutput1);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (initialCheckCount < checkThreshold) {
            initialCheckCount++;
            return false;
        }

        double distError = right.getClosedLoopError(), angleError = right.getClosedLoopError(DriveTrain.DRIVE_SMOOTH_MOTION_SLOT);

        angleError = MercMath.pigeonUnitsToDegrees(angleError);

        boolean isFinished = false;

        boolean isOnTarget = (Math.abs(distError) < moveThresholdTicks &&
                Math.abs(angleError) < DriveTrain.ANGLE_THRESHOLD_DEG);

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
        this.driveTrain.stop();
        this.driveTrain.configVoltage(DriveTrain.NOMINAL_OUT, DriveTrain.PEAK_OUT);
    }
}
