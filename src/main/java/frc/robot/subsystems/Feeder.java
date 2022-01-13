/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap.CAN;
import frc.robot.util.MercMotorController.*;
import frc.robot.util.interfaces.IMercMotorController;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

public class Feeder extends SubsystemBase implements IMercShuffleBoardPublisher {
  
  private IMercMotorController feedWheel;
  private static final double RUN_SPEED = 1.0;

  /**
   * Creates a new Feeder.
   */
  public Feeder() {
    feedWheel = new MercTalonSRX(CAN.FEEDER);
    feedWheel.setInverted(false);
    feedWheel.setNeutralMode(NeutralMode.Brake);
    setName("Feeder");
  }

  public double getRunSpeed() {
    return RUN_SPEED;
  }

  public void runFeeder() {
    feedWheel.setSpeed(RUN_SPEED);
  }

  public void setSpeed(double speed) {
    feedWheel.setSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void publishValues() {
    //SmartDashboard.putNumber("Speed", feedWheel.getSpeed());
  }
}
