// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.ResetMoveHeading;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.ResetEncoders;
import frc.robot.commands.drivetrain.MoveHeadingDerivatives.TrackTarget;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightCamera;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SafeTrackTarget extends SequentialCommandGroup {
  /** Creates a new SafeTrackTarget. */
  public SafeTrackTarget(DriveTrain driveTrain, LimelightCamera limelightCamera) {
    addCommands(new ResetEncoders(driveTrain), new TrackTarget(driveTrain, limelightCamera));
  }
}
