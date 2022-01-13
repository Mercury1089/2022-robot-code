/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.util.MercMotionProfile.ProfileDirection;
import frc.robot.util.MercMotionProfile;
import frc.robot.util.ShuffleDash;
import frc.robot.util.ShuffleDash.Autons;

import frc.robot.commands.drivetrain.*;
import frc.robot.commands.drivetrain.MoveHeadingDerivatives.*;
import frc.robot.commands.elevator.*;
import frc.robot.commands.feeder.*;
import frc.robot.commands.hopper.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.limelightCamera.*;
import frc.robot.commands.shooter.*;

import frc.robot.sensors.Limelight;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.ShootingStyle;

public class AutonSelector extends InstantCommand {
  Autons auton;
  ShuffleDash shuffleDash;
  CommandGroupBase autonCommand;

  DriveTrain driveTrain;
  Elevator elevator;
  Feeder feeder;
  Hopper hopper;
  Intake intake;
  IntakeArticulator intakeArticulator;
  Limelight limelight;
  LimelightCamera limelightCamera;
  Shooter shooter;

  public AutonSelector(ShuffleDash shuffleDash, CommandGroupBase autonCommand, DriveTrain driveTrain, 
    Elevator elevator, Feeder feeder, Hopper hopper, 
    Intake intake, IntakeArticulator intakeArticulator, Limelight limeLight, 
    LimelightCamera limelightCamera, Shooter shooter) {

    super();
    
    this.shuffleDash = shuffleDash;
    this.autonCommand = autonCommand;

    this.driveTrain = driveTrain;
    this.elevator = elevator;
    this.feeder = feeder;
    this.hopper = hopper;
    this.intake = intake;
    this.intakeArticulator = intakeArticulator;
    this.limelight = limelight;
    this.limelightCamera = limelightCamera;
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
        case CENTER_2BALL_RENDEZVOUS:
            initCenter2BallRendezvous();
            break;
        case CENTER_5BALL_RENDEZVOUS:
            initCenter5BallRendezvous();
            break;
        case CENTER_5BALL_TRENCH:
            initCenter5BallTrench();
            break;
        case INITIATION_LINE:
            initInitiationLine();
            break;
        case LEFT_2BALL_TRENCH:
            initLeft2BallTrench();
            break;
        case LEFT_5BALL_TRENCH:
            initLeft5BallTrench();
            break;
        case RIGHT_5BALL_RENDEZVOUS:
            initRight5BallRendezvous();
            break;
        case STEAL_OPPONENT_2BALL:
            initStealOpponent2Ball();
            break;
        default:
    }
  }

  public boolean runsWhenDisabled() {
    return true;
  }
  //Pickup 2 balls from the rendezvous, and then shoot 5
  private void initCenter2BallRendezvous() {
    try {
      autonCommand = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new MoveOnTrajectory(new MercMotionProfile("Center2BallRendezvous", ProfileDirection.BACKWARD), driveTrain),
                    new RunCommand(() -> intakeArticulator.setIntakeOut(), intakeArticulator),
                    new RunIntake(intake),
                    new RunShooterRPMPID(shooter, limelight, ShootingStyle.MANUAL)
                ),
                new ParallelDeadlineGroup(
                    new MoveOnTrajectory(new MercMotionProfile("2BallRendezvousToShoot", ProfileDirection.FORWARD), driveTrain),
                    new RunCommand(() -> intakeArticulator.setIntakeIn(), intakeArticulator)
                ),
                new FullyAutoAimbot(driveTrain, shooter, feeder, hopper, intake, limelight, ShootingStyle.AUTOMATIC)
            );
    } catch (Exception e) {
      System.out.println(e);
    }
  }

  private void initCenter5BallRendezvous() {
    try {

    } catch (Exception e) {
      System.out.println(e);
    }
  }

  private void initCenter5BallTrench() {
    try {

    } catch (Exception e) {
      System.out.println(e);
    }
  }

  private void initInitiationLine() {
    try {
      autonCommand = new SequentialCommandGroup(
                new DriveDistance(-24.0, driveTrain),
                new FullyAutoAimbot(driveTrain, shooter, feeder, hopper, intake, limelight)
      );
    } catch (Exception e) {
      System.out.println(e);
    }
  }

  private void initLeft2BallTrench() {
    try {

    } catch (Exception e) {
      System.out.println(e);
    }
  }

  private void initLeft5BallTrench() {
    try {

    } catch (Exception e) {
      System.out.println(e);
    }
  }

  private void initRight5BallRendezvous() {
    try {

    } catch (Exception e) {
      System.out.println(e);
    }
  }

  private void initStealOpponent2Ball() {
    try {

    } catch (Exception e) {
      System.out.println(e);
    }
  }
}
