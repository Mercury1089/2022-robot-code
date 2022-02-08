    /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LimelightLEDState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.util.MercMath;

public class RotateToTarget extends CommandBase 
{

    private int reTargetCount = 0;
    private boolean isReadyToShoot;
    private Turret turret;
    private double targetHeading;
    private double currentEncoderPos;
    

    public RotateToTarget(Turret turret) {
        setName("RotateToTarget");
        addRequirements(turret);

        this.turret = turret;
    
    }
    

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        
        // turret.getLimelight().setLEDState(LimelightLEDState.ON);
        
        
        targetHeading = turret.getAngleToTarget();
        currentEncoderPos = turret.getCustomTickInDegrees();
        System.out.println("RotateToTarget initialized with angle " + targetHeading);

        turret.setPosition(currentEncoderPos+targetHeading);
        
    }
    
  

    @Override
    public void execute() {
        if (turret.isOnTarget()) {
            reTargetCount++;
        } else {
            reTargetCount = 0;   
        }

        if (reTargetCount > 5) {
            targetHeading = turret.getAngleToTarget();
            currentEncoderPos = turret.getCustomTickInDegrees();
            turret.setPosition(currentEncoderPos+targetHeading);
        }
    }


    @Override
    public boolean isFinished() {

       return turret.isOnTarget() && turret.isAligned();
       
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {

    }
}
