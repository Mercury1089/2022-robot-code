/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.MoveHeadingDerivatives;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LimelightLEDState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.MercMath;

public class RotateToTarget extends DegreeRotate {

    private DriveTrain driveTrain;
    private Limelight limelight;
    private int reTargetCount = 0;
    private boolean isReadyToShoot;

    public RotateToTarget(DriveTrain driveTrain) {
        super(0, driveTrain);
        setName("RotateToTarget");

        this.driveTrain = driveTrain;
        this.limelight = driveTrain.getLimelight();
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();
        this.driveTrain.configVoltage(DriveTrain.NOMINAL_OUT, DriveTrain.PEAK_OUT);
        limelight.setLEDState(LimelightLEDState.ON);
        this.isReadyToShoot = false;
        
        targetHeading = -MercMath.degreesToPigeonUnits(limelight.getTargetCenterXAngle());
        System.out.println("RotateToTarget initialized with angle " + limelight.getTargetCenterXAngle());
        reTargetCount = 0;
        driveTrain.configNeutralDeadband(DriveTrain.ROTATION_NEUTRAL_DEADBAND);
        //driveTrain.configVoltage(0.025, DriveTrain.PEAK_OUT);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        if(driveTrain.isOnTarget()) {
            double checkTarget = limelight.getTargetCenterXAngle();
            if(Math.abs(checkTarget) > DriveTrain.ANGLE_THRESHOLD_DEG && !this.isReadyToShoot) {
                targetHeading = -MercMath.degreesToPigeonUnits(checkTarget);
                driveTrain.resetPigeonYaw();
                onTargetCount = 0;
                reTargetCount++;
            } else {
                // Once isReadyToShoot is true, stop checking limelight and relay in gyro only
                // (This is in case the camera becomes obscured)
                this.isReadyToShoot = true;
            }
        }
        super.execute();
        SmartDashboard.putNumber(driveTrain.getName() + "/" + getName() + "/onTargetCount", onTargetCount);
        SmartDashboard.putNumber(driveTrain.getName() + "/" + getName() + "/reTargetCount", reTargetCount);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.configNeutralDeadband(DriveTrain.NEUTRAL_DEADBAND);
    }
}
