/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class AutoRunHopperBelt extends CommandBase {
  private Hopper hopper;
  private Shooter shooter;
  private DriveTrain driveTrain;

  /**
   * Creates a new AutoRunHopperBelt.
   */
  public AutoRunHopperBelt(Hopper hopper, Shooter shooter, DriveTrain driveTrain){
    addRequirements(hopper);
    this.hopper = hopper;
    this.shooter = shooter;
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.atTargetRpm() && driveTrain.isAligned())
      hopper.setSpeed(hopper.getRunSpeed());
    else
      hopper.setSpeed(0.0);
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
