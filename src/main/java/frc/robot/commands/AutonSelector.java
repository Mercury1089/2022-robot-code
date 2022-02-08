/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArticulator;
import frc.robot.subsystems.LimelightCamera;
import frc.robot.subsystems.Shooter;
import frc.robot.util.ShuffleDash;
import frc.robot.util.ShuffleDash.Autons;

public class AutonSelector extends InstantCommand {
  Autons auton;
  ShuffleDash shuffleDash;
  CommandGroupBase autonCommand;

  DriveTrain driveTrain;
  Elevator elevator;
  Feeder feeder;
  Intake intake;
  IntakeArticulator intakeArticulator;
  Limelight limelight;
  LimelightCamera limelightCamera;
  Shooter shooter;

  public AutonSelector(ShuffleDash shuffleDash, CommandGroupBase autonCommand, DriveTrain driveTrain, 
    Elevator elevator, Feeder feeder, 
    Intake intake, IntakeArticulator intakeArticulator, Limelight limeLight, 
    Shooter shooter) {

    super();
    
    this.shuffleDash = shuffleDash;
    this.autonCommand = autonCommand;

    this.driveTrain = driveTrain;
    this.elevator = elevator;
    this.feeder = feeder;
    this.intake = intake;
    this.intakeArticulator = intakeArticulator;
    this.shooter = shooter;
    
    setName("AutonSelector"); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    auton = shuffleDash.getAuton();
    if(auton == null || auton == Autons.NOTHING) {
      System.out.println("No Auton My Dude");
      return;
    } 
    switch(auton) {
        default:
    }
  }

  public boolean runsWhenDisabled() {
    return true;
  }

}
