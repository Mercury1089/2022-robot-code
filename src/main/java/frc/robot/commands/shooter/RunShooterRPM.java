/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.util.interfaces.IMercMotorController;
import frc.robot.subsystems.DriveTrain;

public class RunShooterRPM extends CommandBase {

  protected IMercMotorController shooterLeft, shooterRight;

  private Shooter shooter;

  /**
   * Creates a new RunShooter.
   */
  public RunShooterRPM(Shooter shooter) {
    super.addRequirements(shooter);
    setName("RunShooterRPM");
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.setVelocity(Math.abs(shooter.getRunRPM()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
