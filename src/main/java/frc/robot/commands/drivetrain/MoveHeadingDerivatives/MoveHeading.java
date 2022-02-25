/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.MoveHeadingDerivatives;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class MoveHeading extends CommandBase {
    protected final int CLOSED_LOOP_TIME_MS = 1;
    protected int onTargetMinCount;
    protected int checkThreshold = 50;

    protected double distance, heading, targetDistance, targetHeading;
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

        onTargetMinCount = 10; // 200 millis

        this.distance = distance;
        this.heading = heading;
        targetDistance = this.distance;
        targetHeading = this.heading;
    }
    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        onTargetCount = 0;
        initialCheckCount = 0;

        this.driveTrain.configPIDSlots(DriveTrain.DRIVE_PID_SLOT, DriveTrain.DRIVE_SMOOTH_MOTION_SLOT);
        targetDistance = distance + this.driveTrain.getRightEncPositionInFeet() * 12; // feet --> inches
        targetHeading = heading + this.driveTrain.getPigeonYawInDegrees();

        driveTrain.moveHeading(targetDistance, targetHeading);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {}

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
      
        

        if (driveTrain.isOnTarget()) {  
            onTargetCount++;
        } else {
            onTargetCount = 0;
        }

        return onTargetCount > onTargetMinCount;

    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        this.driveTrain.stop();
    }
}
