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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap.CAN;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

import frc.robot.sensors.REVColor;
import frc.robot.sensors.REVColor.CargoColor;
import frc.robot.sensors.REVColor.ColorSensorPort;

public class Feeder extends SubsystemBase implements IMercShuffleBoardPublisher {
  
  private TalonSRX feedWheel;
  private static final double RUN_SPEED = 1.0;
  private REVColor colorSensor;

  /**
   * Creates a new Feeder.
   */
  public Feeder(ColorSensorPort colorPort) {
    feedWheel = new TalonSRX(CAN.FEEDER);
    feedWheel.setInverted(false);
    feedWheel.setNeutralMode(NeutralMode.Brake);
    setName("Feeder " + colorPort.toString());
    

    colorSensor = new REVColor(colorPort);
  }

  private void setSpeed(double speed) {
    feedWheel.set(ControlMode.PercentOutput, speed);
  }

  public void runFeeder() {
    this.setSpeed(RUN_SPEED);
  }

  public void stopFeeder() {
    this.setSpeed(0.0);
  }

  public boolean checkCorrectColor(CargoColor allianceColor){
    return colorSensor.getColor() == allianceColor;
  }

  @Override
  public void publishValues() {

    SmartDashboard.putString(getName() + "/Color/Detected", colorSensor.getDetectedColor().toString());
    SmartDashboard.putNumber(getName() + "/Color/Confidence", colorSensor.getConfidence());
    SmartDashboard.putString(getName() + "/Color/ENUM", colorSensor.getColor().toString());
   
    SmartDashboard.putNumber(getName() + "/Color/RGB/Red", colorSensor.getDetectedColor().red * 255);
    SmartDashboard.putNumber(getName() + "/Color/RGB/Green", colorSensor.getDetectedColor().green * 255);
    SmartDashboard.putNumber(getName() + "/Color/RGB/Blue", colorSensor.getDetectedColor().blue * 255);
  }
}
