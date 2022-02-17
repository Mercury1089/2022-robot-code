// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class ScanForTarget extends CommandBase {

  private Turret turret;
  private double encoderAngle;
  private TurretDirection direction;

  /** Creates a new FindTarget. */
  public ScanForTarget(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;


    addRequirements(turret);
    setName("ScanForTarget");

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.encoderAngle = this.turret.getCustomTickInDegrees();

    if (this.encoderAngle > 180) {
      direction = TurretDirection.BACK; // depending on limit switch
    } else if (this.encoderAngle < 180) {
      direction = TurretDirection.FORWARD; // depending on limit switch
    }

  

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    turret.setPosition(direction.turretPosition);

    if (turret.isAtForwardLimit()) {
      direction = TurretDirection.BACK;
    } else if (turret.isAtReverseLimit()) {
      direction = TurretDirection.FORWARD;
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public enum TurretDirection {
    // the position of the turret to rotate to (and current direction of rotation)
    FORWARD(360.0),
    BACK(0.0);

    public final double turretPosition;

    TurretDirection(double turretPosition) {
      this.turretPosition = turretPosition;
    }


  }
}
