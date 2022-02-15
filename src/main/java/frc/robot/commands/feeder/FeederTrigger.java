// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;


public class FeederTrigger extends CommandBase {

  Feeder feederFront, feederBack;
 

  /** Creates a new FeederTrigger. */
  public FeederTrigger(Feeder feederFront, Feeder feederBack) {

    this.feederFront = feederFront;
    this.feederBack = feederBack;
     addRequirements(feederFront);
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!this.feederBack.isBeamBroken()) {
      this.feederBack.setSpeed(0.6);
    } else {
      this.feederBack.setSpeed(0.0);
    }

    if ( (!this.feederFront.isBeamBroken() && !this.feederBack.isBeamBroken())  || 
    (this.feederFront.isBeamBroken() && !this.feederBack.isBeamBroken()) || 
    (!this.feederFront.isBeamBroken() && this.feederBack.isBeamBroken())) {
      this.feederFront.setSpeed(0.6);
    } else {
      this.feederFront.setSpeed(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.feederBack.setSpeed(0.0);
    this.feederFront.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
