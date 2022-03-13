// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.FeedSpeed;


public class LoadFeederTrigger extends CommandBase {

  Feeder feeder;
  Supplier<Boolean> beamCondition;
 
  /** Creates a new FeederTrigger. */
  public LoadFeederTrigger(Feeder feeder, Supplier<Boolean> beamCondition) {

    this.feeder = feeder;
    this.beamCondition = beamCondition;
    setName("LoadFeederTrigger");
    
     addRequirements(feeder);
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}



  @Override
  public void execute() {

   if (beamCondition.get()) {
     feeder.setSpeed(FeedSpeed.LOAD);
   } else {
     feeder.setSpeed(FeedSpeed.STOP);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.feeder.setSpeed(FeedSpeed.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
