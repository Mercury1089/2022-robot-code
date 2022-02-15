/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.REVColorMux.I2CMUX;
import frc.robot.sensors.REVColorMux.REVColor;

import frc.robot.sensors.REVColorMux.REVColor.ColorSensorID;

public class Feeder extends SubsystemBase {
  
  private VictorSPX feedWheel;
  private static final double RUN_SPEED = 1.0;
  private REVColor colorSensor;
  private DigitalInput breakBeamSensor;
  private int dioPort;

  /**
   * Creates a new Feeder.
   */
  public Feeder(ColorSensorID colorSensorID, BreakBeamDIO DIOPort, int motorControllerID, I2CMUX mux) {

    if (DIOPort == BreakBeamDIO.FRONT) {
      dioPort = 0;
    } else if (DIOPort == BreakBeamDIO.BACK) {
      dioPort = 1;
    }

    
    breakBeamSensor = new DigitalInput(dioPort); 
    feedWheel = new VictorSPX(motorControllerID);
    feedWheel.configFactoryDefault();
    feedWheel.setInverted(true);
    feedWheel.setNeutralMode(NeutralMode.Brake);
    setName("Feeder " + DIOPort.toString());
    

    colorSensor = new REVColor(colorSensorID, mux);
  }

  public void setSpeed(double speed) {
    feedWheel.set(ControlMode.PercentOutput, speed);
  }

  public void runFeeder() {
    this.setSpeed(RUN_SPEED);
  }

  public void stopFeeder() {
    this.setSpeed(0.0);
  }

  public boolean isCorrectColor() {
    return colorSensor.getColor() ==  DriverStation.getAlliance();
  }

  public boolean isBeamBroken() {
    return !breakBeamSensor.get(); 
  }

  public enum BreakBeamDIO {
    FRONT,
    BACK
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    
    builder.setActuator(true); // Only allow setting values when in Test mode
    builder.setSafeState(() -> setSpeed(0)); // Provide method to make the subsystem safe
    builder.addStringProperty("Color/Detected", () -> colorSensor.getDetectedColor().toString(), null);
    builder.addDoubleProperty("Color/Confidence", () -> colorSensor.getConfidence(), null);
    builder.addStringProperty("Color/ENUM", () -> colorSensor.getColor().toString(), null);
    builder.addStringProperty("Color/SameAllianceColor", () -> "" + isCorrectColor(), null);
    
    builder.addDoubleProperty("Color/RGB/Red", () -> colorSensor.getDetectedColor().red * 255, null);
    builder.addDoubleProperty("Color/RGB/Green", () -> colorSensor.getDetectedColor().green * 255, null);
    builder.addDoubleProperty("Color/RGB/Blue", () -> colorSensor.getDetectedColor().blue * 255, null);

    builder.addBooleanProperty("isBeamBroken", () -> isBeamBroken(), null);
  }
}
