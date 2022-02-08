/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drivetrain.MoveHeadingDerivatives.StayOnTarget;
import frc.robot.commands.feeder.AutoFeedBalls;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.ShootingStyle;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FullyAutoAimbot extends ParallelCommandGroup {
  /**
   * Creates a new ShootFullyAutomatic.
   */
  
  public FullyAutoAimbot(DriveTrain driveTrain, Shooter shooter, Feeder feeder, Intake intake, Limelight limelight, ShootingStyle shootingStyle) {
    //Rotates to target and revs shooter to target rpm, THEN it runs the feeder and hopper
    super(new StayOnTarget(driveTrain, shootingStyle),
          new RunShooterRPMPID(shooter, limelight, shootingStyle),
          new AutoFeedBalls(feeder, intake, shooter, driveTrain, shootingStyle));
  }
  
  /*
  public FullyAutoAimbot(DriveTrain driveTrain, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, Limelight limelight, ShootingStyle shootingStyle) {
    //Rotates to target and revs shooter to target rpm, THEN it runs the feeder and hopper
    super(new StayOnTarget(driveTrain, shootingStyle),
          new RunShooterRPMPID(shooter, limelight, shootingStyle),
          new AutoFeedBalls(feeder, hopper, intake, shooter, driveTrain, shootingStyle),
          new RunIntake(intake, 0.7));
  }
  */
  public FullyAutoAimbot(DriveTrain driveTrain, Shooter shooter, Feeder feeder, Intake intake, Limelight limelight) {
    //Rotates to target and revs shooter to target rpm, THEN it runs the feeder and hopper
    this(driveTrain, shooter, feeder, intake, limelight, ShootingStyle.AUTOMATIC);
  }
}
