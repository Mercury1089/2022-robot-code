// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class LoadFeeder extends CommandBase {

  private Feeder feeder;
  private Supplier<Boolean> beamIsBroken;
  private double runSpeed;
  private double shootSpeed;
  /** Creates a new RunFeeder. */
  public LoadFeeder(Feeder feeder, Supplier<Boolean> beamIsBroken) {
    this.beamIsBroken = beamIsBroken;
    this.feeder = feeder;
    addRequirements(feeder);
    setName("LoadFeeder");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.feeder.setSpeed(0.60);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.feeder.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.beamIsBroken.get();
  }
}
