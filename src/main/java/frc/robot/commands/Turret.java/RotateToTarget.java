    /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.MoveHeadingDerivatives;
package frc.robot.commands.Turret.java;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LimelightLEDState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.MercMath;

public class RotateToTarget extends DegreeRotate {

    private Limelight limelight;
    private int reTargetCount = 0;
    private boolean isReadyToShoot;

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();
        limelight.setLEDState(LimelightLEDState.ON);
        this.isReadyToShoot = false;
        
        targetHeading = limelight.getTargetCenterXAngle();
        System.out.println("RotateToTarget initialized with angle " + limelight.getTargetCenterXAngle());
        reTargetCount = 0;
    }
    
    /**
     * Get the current angle error for closed loop driving
     * @return the error in degrees
     */
    public double getAngleError() {
        return shooterLeft.getClosedLoopError(AUXILIARY_LOOP);
    }
  
    public double getTurnAngle(){

    }
}
@Override
    public void execute() {

    if(Turret.isOnTarget()) {
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
    }
}
