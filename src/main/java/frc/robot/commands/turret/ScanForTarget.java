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
    addRequirements(turret);
    setName("ScanForTarget");

    this.turret = turret;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.encoderAngle = turret.getCustomTickInDegrees();

    if (this.encoderAngle > 180) {
      direction = TurretDirection.BACK; // depending on limit switch
    } else if (this.encoderAngle < 180) {
      direction = TurretDirection.FORWARD; // depending on limit switch
    }

  

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (direction == TurretDirection.BACK) {
      turret.setPosition(0.0);

    } else if (direction == TurretDirection.FORWARD) {
      turret.setPosition(360.0);
    }

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
    FORWARD,
    BACK
  }
}
