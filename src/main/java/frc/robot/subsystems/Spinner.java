/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap.CAN;
import frc.robot.sensors.REVColor;
import frc.robot.sensors.REVColor.ControlPanelColor;

import frc.robot.util.MercMotorController.*;
import frc.robot.util.interfaces.IMercMotorController;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

public class Spinner extends SubsystemBase implements IMercShuffleBoardPublisher {
  
  private IMercMotorController spinController;
  private REVColor colorSensor;
  private final double RUN_SPEED = -0.2;
  private double colorsCrossed;

  /**
   * Creates a new Spinner.
   */
  public Spinner() {
    super();
    setName("Spinner");
    spinController = new MercTalonSRX(CAN.SPINNER);
    colorSensor = new REVColor();
    SmartDashboard.putNumber("Spin speed", 0.0);
  }

  public double getRunSpeed() {
    return RUN_SPEED;
  }

  public double getEncTicks() {
    return spinController.getEncTicks();
  }

  public void setSpeed(double speed) {
    spinController.setSpeed(speed);
  }

  public void setColorsCrossed(double colorsCrossed) {
    this.colorsCrossed = colorsCrossed;
  }

  public REVColor getColorSensor() {
    return colorSensor;
  }

  public Color getDetectedColor() {
    return colorSensor.getDetectedColor();
  }

  
  public ControlPanelColor getFmsColor() {
    String fmsColor = DriverStation.getInstance().getGameSpecificMessage();
        if(fmsColor.length() > 0)
            switch(fmsColor.charAt(0)) {
                case 'R':
                    return ControlPanelColor.BLUE;
                case 'G':
                  return ControlPanelColor.YELLOW;
                case 'B':
                  return ControlPanelColor.RED;
                case 'Y':
                  return ControlPanelColor.GREEN;
                default:
                  return ControlPanelColor.UNKNOWN;
            }
        return ControlPanelColor.UNKNOWN;
  }

  public ControlPanelColor getNextColor(ControlPanelColor currentColor) {
    if(currentColor == ControlPanelColor.RED)
        return ControlPanelColor.GREEN;
      else if(currentColor == ControlPanelColor.GREEN)
        return ControlPanelColor.BLUE;
      else if(currentColor == ControlPanelColor.BLUE)
        return ControlPanelColor.YELLOW;
      else if(currentColor == ControlPanelColor.YELLOW)
        return ControlPanelColor.RED;
    return ControlPanelColor.UNKNOWN;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void publishValues() {
    SmartDashboard.putNumber(getName() + "/Ticks", getEncTicks());
    SmartDashboard.putString(getName() + "/Color/Detected", colorSensor.get().toString());
    SmartDashboard.putNumber(getName() + "/Color/Confidence", colorSensor.getConfidence());
    
    SmartDashboard.putNumber(getName() + "/Color/RGB/Red", colorSensor.getDetectedColor().red);
    SmartDashboard.putNumber(getName() + "/Color/RGB/Green", colorSensor.getDetectedColor().green);
    SmartDashboard.putNumber(getName() + "/Color/RGB/Blue", colorSensor.getDetectedColor().blue);
    SmartDashboard.putNumber("Color Changes", colorsCrossed);

    SmartDashboard.putString(getName() + "Current Command", getCurrentCommand().getName());
  }
}
