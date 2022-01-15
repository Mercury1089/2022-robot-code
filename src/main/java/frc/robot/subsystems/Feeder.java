/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap.CAN;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

public class Feeder extends SubsystemBase implements IMercShuffleBoardPublisher {
  
  private TalonSRX feedWheel;
  private static final double RUN_SPEED = 1.0;

  /**
   * Creates a new Feeder.
   */
  public Feeder() {
    feedWheel = new TalonSRX(CAN.FEEDER);
    feedWheel.setInverted(false);
    feedWheel.setNeutralMode(NeutralMode.Brake);
    setName("Feeder");
  }

  public double getRunSpeed() {
    return RUN_SPEED;
  }

  public void runFeeder() {
    this.setSpeed(RUN_SPEED);
  }

  public void setSpeed(double speed) {
    feedWheel.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void publishValues() {
    //SmartDashboard.putNumber("Speed", feedWheel.getSpeed());
  }
}
