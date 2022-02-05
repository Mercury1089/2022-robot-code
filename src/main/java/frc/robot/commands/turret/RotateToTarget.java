    /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.util.MercMath;

public class RotateToTarget extends CommandBase {

    private Turret turret;
    private int onTargetCount = 0, reTargetCount = 0;
    private boolean isReadyToShoot;
    private double targetHeading;

    public RotateToTarget(Turret turret) {
        this.turret = turret;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        this.isReadyToShoot = false;
        
        targetHeading = turret.getAngleToTarget();
        System.out.println("RotateToTarget initialized with angle " + targetHeading);
        reTargetCount = 0;
    }
    
    /**
     * Get the current angle error for closed loop driving
     * @return the error in degrees
     */
    public double getAngleError() {
        return turret.getAngleError();
    }

    @Override
    public void execute() {

        if(turret.isOnTarget()) {
            double checkTarget = turret.getAngleToTarget();
            if(Math.abs(checkTarget) > DriveTrain.ANGLE_THRESHOLD_DEG && !this.isReadyToShoot) {
                targetHeading = -MercMath.degreesToPigeonUnits(checkTarget);
                onTargetCount = 0;
                reTargetCount++;
            } else {
                // Once isReadyToShoot is true, stop checking limelight and relay in gyro only
                // (This is in case the camera becomes obscured)
                this.isReadyToShoot = true;
            }
        }
    }
}
