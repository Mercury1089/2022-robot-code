/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.MoveHeadingDerivatives;

import frc.robot.subsystems.DriveTrain;
//import frc.robot.commands.drivetrain.MoveOnPath.MPDirection;

public class PigeonFollower extends MoveHeading {
  private double currDistAngle;
  private double[][] distAngle;
  private DriveTrain driveTrain;

  public PigeonFollower(double distance, double[][] distAngle, DriveTrain driveTrain) {
    super(distance, 0, driveTrain);
    setName("PigeonFollower");
    this.driveTrain = driveTrain;
    this.distAngle = distAngle;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    //targetHeading = 
    //right.set(ControlMode.Position, distance, DemandType.AuxPID, targetHeading);
    //left.follow(right, FollowerType.AuxOutput1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
