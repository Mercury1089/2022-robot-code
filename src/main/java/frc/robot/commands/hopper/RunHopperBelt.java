/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class RunHopperBelt extends CommandBase {

  private Hopper hopper;

  /**
   * Creates a new runHopperBelt.
   */
  public RunHopperBelt(Hopper hopper) {
    addRequirements(hopper);
    setName("RunHopperBelt");
    this.hopper = hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.setSpeed(hopper.getRunSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
