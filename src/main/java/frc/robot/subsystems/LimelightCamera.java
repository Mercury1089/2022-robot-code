/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.Limelight;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

public class LimelightCamera extends SubsystemBase implements IMercShuffleBoardPublisher{
  
  private Limelight limelight;

  /**
   * Creates a new LimelightCamera.
  */
  public LimelightCamera() {
    super();
    this.limelight = new Limelight();
    setName("LimelightCamera");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Limelight getLimelight(){
    return this.limelight;
  }

  @Override
  public void publishValues() {
    SmartDashboard.putNumber(getName() + "/DistanceFromTarget", getLimelight().getRawAreaDistance());
    //SmartDashboard.putNumber(getName() + "/FromAngle", limelight.calcDistFromAngle());
    //SmartDashboard.putNumber(getName() + "/FromArea", limelight.calcDistFromArea());
    //SmartDashboard.putNumber(getName() + "/FromVert", limelight.calcDistFromVert());
    //SmartDashboard.putNumber(getName() + "/CenterXAngle", limelight.getTargetCenterXAngle());
    SmartDashboard.putBoolean(getName() + "/TargetAcquired", limelight.getTargetAcquired());
    /*
    SmartDashboard.putNumber(getName() + "From Horiz", limelight.calcDistFromHoriz());
    */
  
  }
}
