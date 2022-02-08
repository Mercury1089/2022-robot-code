/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.ShootingStyle;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoFeedBalls extends CommandBase {
  private Feeder feeder;
  private Shooter shooter;
  private DriveTrain driveTrain;
  private ShootingStyle shootingStyle;
  private Intake intake;
  private boolean started;

  /**
   * Creates a new AutoFeedBalls.
   */
  public AutoFeedBalls(Feeder feeder, Intake intake, Shooter shooter, DriveTrain driveTrain, ShootingStyle shootingStyle) {
    //addRequirements(feeder, hopper);
    addRequirements(feeder, intake);
    this.feeder = feeder;
    this.intake = intake;
    this.shooter = shooter;
    this.driveTrain = driveTrain;
    this.shootingStyle = shootingStyle;
  }

  public AutoFeedBalls(Feeder feeder, Intake intake, Shooter shooter, DriveTrain driveTrain) {
    this(feeder, intake, shooter, driveTrain, ShootingStyle.AUTOMATIC);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    started = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(driveTrain.isReadyToShoot() && shooter.isReadyToShoot()){
    //   hopper.runHopper();
    //   feeder.runFeeder();
    //   started = true;
    // } else {
    //   hopper.stopHopper();
    //   feeder.stopFeeder();
    // }
    // if (started) {
    //   intake.runIntakeRoller(0.7);
    //   intake.runAgitator();
    // } else {
    //   intake.stopIntakeRoller();
    //   intake.stopAgitator();
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopFeeder();
    intake.stopAgitator();
    intake.stopIntakeRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
