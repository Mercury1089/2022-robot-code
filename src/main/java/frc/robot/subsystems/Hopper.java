/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap.CAN;

import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

public class Hopper extends SubsystemBase implements IMercShuffleBoardPublisher {
  
  private VictorSPX hopperBelt;
  private final double RUN_SPEED;

  /**
   * Creates a new Hopper.
   */
  public Hopper() {
    RUN_SPEED = -0.5;

    hopperBelt = new VictorSPX(CAN.HOPPER_BELT);
    hopperBelt.setNeutralMode(NeutralMode.Brake);

    setName("Hopper");
  }

  public void setSpeed(double speed) {
    hopperBelt.set(ControlMode.PercentOutput, speed);
  }

  public void stopHopper() {
    setSpeed(0.0);
  }

  public void runHopper(){
    setSpeed(RUN_SPEED);
  }

  public double getRunSpeed() {
    return RUN_SPEED;
  }

  @Override
  public void publishValues() {

  }
}
