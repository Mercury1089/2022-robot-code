/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import java.io.FileNotFoundException;
import java.util.List;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.MercMath;
import frc.robot.util.MercMotionProfile;
import frc.robot.util.MercPathLoader;

public class MoveOnTrajectory extends CommandBase {

  private DriveTrain driveTrain;
  private List<TrajectoryPoint> trajectoryPoints;
  private String pathName;
  private MercMotionProfile profile;
  private BufferedTrajectoryPointStream buffer;

  public MoveOnTrajectory(String path, DriveTrain driveTrain) throws FileNotFoundException{
    addRequirements(driveTrain);
    setName("MoveOn " + path + "Path");

    pathName = path;
    this.driveTrain = driveTrain;
    trajectoryPoints = MercPathLoader.loadPath(pathName);
    buffer = new BufferedTrajectoryPointStream();
  }

  public MoveOnTrajectory(MercMotionProfile profile, DriveTrain driveTrain) throws FileNotFoundException {
    this.profile = profile;
    this.driveTrain = driveTrain;
    this.pathName = this.profile.getName();
    
    addRequirements(driveTrain);
    setName("MoveOn " + pathName + "Path");
  
    trajectoryPoints = profile.getTrajectoryPoints();
    buffer = new BufferedTrajectoryPointStream();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (trajectoryPoints == null)
      DriverStation.reportError("No trajectory to load", false);

    // Reconfigure driveTrain settings
    driveTrain.configPIDSlots(DriveTrain.DRIVE_MOTION_PROFILE_SLOT, DriveTrain.DRIVE_SMOOTH_MOTION_SLOT);
    driveTrain.resetPigeonYaw();
    driveTrain.resetEncoders();
    buffer.Clear();

    int minTime = 0;
    for(TrajectoryPoint point : trajectoryPoints) {
      minTime = Math.min(point.timeDur, minTime);

      double encoderTicks = MercMath.inchesToEncoderTicks(this.driveTrain.getDistanceTarget());
      double auxPos = MercMath.degreesToPigeonUnits(this.driveTrain.getAngleTarget());
      TrajectoryPoint newPoint = new TrajectoryPoint();
      newPoint.timeDur = point.timeDur;
      newPoint.velocity = point.velocity;
      newPoint.auxiliaryPos = point.auxiliaryPos + auxPos;
      newPoint.position = point.position + encoderTicks;
      newPoint.zeroPos = point.zeroPos;
      newPoint.profileSlotSelect0 = point.profileSlotSelect0;
      newPoint.profileSlotSelect1 = point.profileSlotSelect1;
      newPoint.useAuxPID = point.useAuxPID;
      newPoint.isLastPoint = point.isLastPoint;

      buffer.Write(point);
    }
    driveTrain.moveOnTrajectory(buffer, minTime);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      DriverStation.reportError(getName() + " is interrupted", false);
    }
    driveTrain.clearTrajectory();
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrain.isTrajectoryFinished();
  }
}
