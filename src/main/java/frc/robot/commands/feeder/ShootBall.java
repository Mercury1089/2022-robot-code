// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder.FeedSpeed;

public class ShootBall extends CommandBase {

  Feeder backFeeder;
  Shooter shooter;
  /** Creates a new ShootBall. */
  public ShootBall(Feeder backFeeder,  Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(backFeeder);
    setName("ShootBall");
    this.backFeeder = backFeeder;
    this.shooter = shooter;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.backFeeder.setIsShooting(true);
    this.backFeeder.setSpeed(FeedSpeed.SHOOT);
    this.shooter.incrementShootCount();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.backFeeder.setSpeed(FeedSpeed.STOP);

    this.backFeeder.setIsShooting(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !this.shooter.hasBall() && !this.backFeeder.hasBall();
  }
}
