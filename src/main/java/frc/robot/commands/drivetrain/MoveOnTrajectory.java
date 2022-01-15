/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveTrainSide;

import frc.robot.util.MercMotionProfile;
import frc.robot.util.MercPathLoader;
import frc.robot.util.MercMotionProfile.ProfileDirection;

public class MoveOnTrajectory extends CommandBase {

  private DriveTrain driveTrain;
  private BaseMotorController left, right;
  private MotionProfileStatus statusRight;
  private List<TrajectoryPoint> trajectoryPoints;
  private String pathName;
  private MercMotionProfile profile;
  private BufferedTrajectoryPointStream buffer;

  public MoveOnTrajectory(String path, DriveTrain driveTrain) throws FileNotFoundException{
    addRequirements(driveTrain);
    setName("MoveOn " + path + "Path");

    pathName = path;
    this.driveTrain = driveTrain;
    statusRight = new MotionProfileStatus();
    trajectoryPoints = MercPathLoader.loadPath(pathName);
    buffer = new BufferedTrajectoryPointStream();

    left = this.driveTrain.getLeftLeader();
    right = this.driveTrain.getRightLeader();
  }

  public MoveOnTrajectory(MercMotionProfile profile, DriveTrain driveTrain) throws FileNotFoundException {
    this.profile = profile;
    this.driveTrain = driveTrain;
    this.pathName = this.profile.getName();
    
    addRequirements(driveTrain);
    setName("MoveOn " + pathName + "Path");
  
    statusRight = new MotionProfileStatus();
    trajectoryPoints = profile.getTrajectoryPoints();
    buffer = new BufferedTrajectoryPointStream();

    left = this.driveTrain.getLeftLeader();
    right = this.driveTrain.getRightLeader();
  }

  public MoveOnTrajectory(DriveTrain driveTrain, String ... paths) {
    this.driveTrain = driveTrain;
    this.pathName = "Many";

    addRequirements(driveTrain);
    setName("MoveOn " + pathName + "Path");

    statusRight = new MotionProfileStatus();
    trajectoryPoints = new ArrayList<TrajectoryPoint>();
    buffer = new BufferedTrajectoryPointStream();

    for(String path: paths) {
      ProfileDirection direction = path.charAt(0) == 'F' ? ProfileDirection.FORWARD : ProfileDirection.BACKWARD;
      MercMotionProfile profile = new MercMotionProfile(path, direction, 0, false);
      trajectoryPoints.addAll(profile.getTrajectoryPoints());
    }
    trajectoryPoints.get(trajectoryPoints.size() - 1).isLastPoint = true;
    
    left = this.driveTrain.getLeftLeader();
    right = this.driveTrain.getRightLeader();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (trajectoryPoints == null)
      DriverStation.reportError("No trajectory to load", false);
    if (!driveTrain.isInMotionMagicMode())
      driveTrain.initializeMotionMagicFeedback();

    reset();
    fillBuffer();
    left.follow(right, FollowerType.AuxOutput1);
    right.startMotionProfile(buffer, 20, ControlMode.MotionProfileArc);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //timeDuration = statusRight.timeDurMs / 2;
    //if(timeDuration < 1)
    //  timeDuration = 1;
    //right.changeMotionControlFramePeriod(timeDuration);
    right.getMotionProfileStatus(statusRight);
    SmartDashboard.putNumber("Primary PID Error", right.getClosedLoopError(0));
    SmartDashboard.putNumber("Aux PID Error", right.getClosedLoopError(1));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      DriverStation.reportError(getName() + " is interrupted", false);
    }
    right.clearMotionProfileTrajectories();

    driveTrain.stop();
    //reset();
    driveTrain.configVoltage(DriveTrain.NOMINAL_OUT, DriveTrain.PEAK_OUT);
    driveTrain.setNeutralMode(NeutralMode.Brake);
    ((BaseTalon)right).setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, RobotMap.CTRE_TIMEOUT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return statusRight.activePointValid && statusRight.isLast && isRunning;
    return right.isMotionProfileFinished();
  }

  // Feeds TalonSRX with trajectory points
  public void fillBuffer() {
    buffer.Clear();
    for(TrajectoryPoint point : trajectoryPoints) {
      buffer.Write(point);
    }
  }

  // Resets values to rerun command
  private void reset() {
    // Reset flags and motion profile modes
    /*
    right.set(ControlMode.MotionProfileArc, SetValueMotionProfile.Disable.value);
    right.configMotionProfileTrajectoryPeriod(0, RobotMap.CTRE_TIMEOUT);
    */
    // Reconfigure driveTrain settings
    driveTrain.configPIDSlots(DriveTrainSide.RIGHT, DriveTrain.DRIVE_MOTION_PROFILE_SLOT, DriveTrain.DRIVE_SMOOTH_MOTION_SLOT);
    driveTrain.setNeutralMode(NeutralMode.Brake);
    driveTrain.resetPigeonYaw();
    driveTrain.resetEncoders();

    int halfFramePeriod = MercPathLoader.getMinTime() / 2;
    if(halfFramePeriod < 1)
      halfFramePeriod = 1;
    right.changeMotionControlFramePeriod(halfFramePeriod);
    right.configAuxPIDPolarity(false);
  }
}
