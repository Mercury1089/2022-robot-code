/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;

public class RunSpinner extends CommandBase {
  
  private Spinner spinner;
  
  /**
   * Creates a new RunSpinner.
   */
  public RunSpinner(Spinner spinner) {
    addRequirements(spinner);
    setName("RunSpinner");
    this.spinner = spinner;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spinner.setSpeed(spinner.getRunSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spinner.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
