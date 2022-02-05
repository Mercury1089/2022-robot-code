/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.MoveHeadingDerivatives;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.MercMath;

public class RotateToTarget extends DegreeRotate {

    private DriveTrain driveTrain;
    private int reTargetCount = 0;
    private boolean isReadyToShoot;

    public RotateToTarget(DriveTrain driveTrain) {
        super(0, driveTrain);
        setName("RotateToTarget");

        this.driveTrain = driveTrain;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();
        this.driveTrain.configVoltage(DriveTrain.NOMINAL_OUT, DriveTrain.PEAK_OUT);
        this.isReadyToShoot = false;
        
        reTargetCount = 0;
        driveTrain.configNeutralDeadband(DriveTrain.ROTATION_NEUTRAL_DEADBAND);
        //driveTrain.configVoltage(0.025, DriveTrain.PEAK_OUT);
    }

    // Called repeatedly when this Command is scheduled to run

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.configNeutralDeadband(DriveTrain.NEUTRAL_DEADBAND);
    }
}
