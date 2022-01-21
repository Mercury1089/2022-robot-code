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
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap.CAN;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

import frc.robot.sensors.REVColor;

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
    ColorMatch colorMatcher = new ColorMatch();
    Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    Color kRedTarget = new Color(0.561, 0.232, 0.114);
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kRedTarget);

    REVColor sensor = new REVColor();
  }

  private void setSpeed(double speed) {
    feedWheel.set(ControlMode.PercentOutput, speed);
  }

  //public boolean checkColor() {
    //Color match = sensor.
  //}

  public void runFeeder() {
    this.setSpeed(RUN_SPEED);
  }

  public void stopFeeder() {
    this.setSpeed(0.0);
  }

  @Override
  public void publishValues() {
    //SmartDashboard.putNumber("Speed", feedWheel.getSpeed());
  }
}
