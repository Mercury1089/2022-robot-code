// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArticulator;
import frc.robot.subsystems.Intake.IntakeSpeed;

public class RobotFullIntakeUp extends CommandBase {

  Intake intake;
  Feeder feederFront, feederBack;
  IntakeArticulator intakeArticulator;
  /** Creates a new RobotFullIntakeUp. */
  public RobotFullIntakeUp(Intake intake, IntakeArticulator intakeArticulator, Feeder feederFront, Feeder feederBack) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intake = intake;
    this.feederBack = feederBack;
    this.feederFront = feederFront;
    this.intakeArticulator = intakeArticulator;
    addRequirements(intake, intakeArticulator);
    setName("RobotFullIntakeUp");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (this.feederFront.hasBall() && this.feederBack.hasBall()) {
        this.intakeArticulator.setIntakeIn();
        this.intake.setSpeed(IntakeSpeed.STOP);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
