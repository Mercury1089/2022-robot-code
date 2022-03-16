// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class WaitForTarget extends CommandBase {

  Shooter shooter;
  Turret turret;
  /** Creates a new WaitForTarget. */
  public WaitForTarget(Shooter shooter, Turret turret) {
    this.shooter = shooter;
    this.turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, turret);

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.turret.setPosition(0);
    this.shooter.setVelocity(Shooter.STEADY_RPM);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.turret.isTargetAcquired();
  }
}
