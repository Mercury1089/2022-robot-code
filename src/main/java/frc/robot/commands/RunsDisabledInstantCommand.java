// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * InstantCommand that can run while the robot is disabled.
 */
public class RunsDisabledInstantCommand extends InstantCommand {

  public RunsDisabledInstantCommand() {
    super();
  }
  public RunsDisabledInstantCommand(Runnable toRun, Subsystem... requirements) {
    super(toRun, requirements);
  }

  // This instant command can run disabled
  public boolean runsWhenDisabled(){
    return true;
  }
}
