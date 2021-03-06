/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LimelightLEDState;
import frc.robot.subsystems.Shooter;

public class RunShooterRPMPID extends CommandBase {

  private Shooter shooter;
  private Limelight limelight;

  /**
   * Creates a new RunShooter.
   */
  public RunShooterRPMPID(Shooter shooter, Limelight limelight) {
    addRequirements(shooter);
    setName("RunShooterRPMPID");
    this.shooter = shooter;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setLEDState(LimelightLEDState.ON);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocity(Math.abs(shooter.getVelocityToTarget()));
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
