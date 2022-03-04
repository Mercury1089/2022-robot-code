    /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class RotateToTarget extends CommandBase 
{

    private Turret turret;
    private double targetHeading;
    private double currentEncoderPos;
    private double OFFSET = 0;
    

    public RotateToTarget(Turret turret) {
        this(turret, 0);
    
    }

    public RotateToTarget(Turret turret, double offset) {
        OFFSET = offset;
        setName("RotateToTarget");
        addRequirements(turret);
        this.turret = turret;
    }
    

    // Called just before this Command runs the first time
    @Override
    public void initialize() {        
        targetHeading = turret.getAngleToTarget(); // limelight's centerX angle
        currentEncoderPos = turret.getCustomTickInDegrees(); // turret's current rotation 
        System.out.println("RotateToTarget initialized with angle " + targetHeading);

        double finalPos = currentEncoderPos + targetHeading + OFFSET;
        turret.setPosition(finalPos);
    }
    
  

    @Override
    public void execute() {
        double newHeading = turret.getAngleToTarget();

        if (newHeading != targetHeading) {
            targetHeading = newHeading;
            currentEncoderPos = turret.getCustomTickInDegrees();
            
            double finalPos = currentEncoderPos + targetHeading + OFFSET;
            turret.setPosition(finalPos);
        }
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {

    }
}
