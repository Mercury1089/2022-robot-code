/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


@SuppressWarnings("all")
public class ShootAccurately extends CommandBase {

  private Shooter shooter;
  private double distance, targetAngle;
  /**
   * Creates a new ShootAccurately.
   */
  public ShootAccurately(double distance, double targetAngle, Shooter shooter) {
    super.addRequirements(shooter);
    setName("ShootAccurately");
    this.shooter = shooter;

    this.distance = distance;
    this.targetAngle = targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
