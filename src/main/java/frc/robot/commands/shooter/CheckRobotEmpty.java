// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class CheckRobotEmpty extends CommandBase {

  Feeder frontFeeder, backFeeder;
  Shooter shooter;
  double counter = 0;
  /** Creates a new CheckRobotEmpty. */
  public CheckRobotEmpty(Feeder frontFeeder, Feeder backFeeder, Shooter shooter) {


    this.frontFeeder = frontFeeder;
    this.backFeeder = backFeeder;
    this.shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!frontFeeder.hasBall() && !backFeeder.hasBall() && !shooter.hasBall()) {
      counter += 1;
    } else {
      counter = 0;
    }
    if (counter > 10) {
      return true;
    }
    return false;
  }
}
