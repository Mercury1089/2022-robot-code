/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.rmi.server.RemoteObjectInvocationHandler;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private CargoColor allianceColor;
  private DigitalInput breakBeamSensor;
  private int dioPort;

  /**
   * Creates a new Feeder.
   */
  public Feeder(ColorSensorPort colorPort, CargoColor alliance, BreakBeamDIO DIOPort) {

    if (DIOPort == BreakBeamDIO.FRONT) {
      dioPort = 0;
    } else if (DIOPort == BreakBeamDIO.BACK) {
      dioPort = 1;
    }
    breakBeamSensor = new DigitalInput(dioPort); // change channel
    allianceColor = alliance;
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

  public boolean isCorrectColor() {
    return colorSensor.getColor() == allianceColor;
  }

  public boolean beamIsBroken() {
    return breakBeamSensor.get(); //might need to swap
  }

  

  public enum BreakBeamDIO {
    FRONT,
    BACK
  }

  @Override
  public void publishValues() {

    SmartDashboard.putString(getName() + "/Color/Detected", colorSensor.getDetectedColor().toString());
    SmartDashboard.putNumber(getName() + "/Color/Confidence", colorSensor.getConfidence());
    SmartDashboard.putString(getName() + "/Color/ENUM", colorSensor.getColor().toString());
    SmartDashboard.putString(getName() + "/Color/SameAllianceColor", "" + isCorrectColor());
   
    SmartDashboard.putNumber(getName() + "/Color/RGB/Red", colorSensor.getDetectedColor().red * 255);
    SmartDashboard.putNumber(getName() + "/Color/RGB/Green", colorSensor.getDetectedColor().green * 255);
    SmartDashboard.putNumber(getName() + "/Color/RGB/Blue", colorSensor.getDetectedColor().blue * 255);
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

    builder.addBooleanProperty("breakBeamSensor", () -> beamIsBroken(), null);
  }
}
