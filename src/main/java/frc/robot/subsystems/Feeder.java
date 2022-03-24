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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.REVColorMux.REVColor;

public class Feeder extends SubsystemBase {
  
  public enum FeedSpeed{
    STOP(0.0), // stop running feeder motors
    LOAD(0.60), // run feeder motors inward
    EJECT(-0.60), // run feeder motors outward
    SHOOT(1.00); // run feeder motors full speed inwards

    public final double speed; 

        /**
         * Creates the feeder speed.
         *
         * @param speed speed, in PercentOutput
         */
        FeedSpeed(double speed) {
            this.speed = speed;
        }
  }

  private VictorSPX feedWheel;
  private REVColor colorSensor;
  private DigitalInput breakBeamSensor;
  private int dioPort;
  private boolean isShooting;
  private boolean hasColorSensor;

  /**
   * Creates a new Feeder.
   */
  public Feeder(boolean hasColorSensor, BreakBeamDIO DIOPort, int motorControllerID) {

    dioPort = DIOPort.dioPort;
    hasColorSensor = hasColorSensor;

    
    breakBeamSensor = new DigitalInput(dioPort); 
    feedWheel = new VictorSPX(motorControllerID);
    feedWheel.configFactoryDefault();
    feedWheel.setInverted(false);
    feedWheel.setNeutralMode(NeutralMode.Brake);
    setName("Feeder " + DIOPort.toString());
    
    if (hasColorSensor) {
      colorSensor = new REVColor();
    } else {
      colorSensor = null;
    }
    
    isShooting = false;
  }

  public void setSpeed(FeedSpeed feedSpeed) {
    feedWheel.set(ControlMode.PercentOutput, feedSpeed.speed);
  }

  public boolean isCorrectColor() {

    
    // is detected color of ball is same as FMS' alliance color 
    // (and make sure there's actually a color being picked up)
    return colorSensor.getColor() ==  DriverStation.getAlliance() &&
    colorSensor.getColor() != DriverStation.Alliance.Invalid;
  }

  public BallMatchesAlliance whoseBall() {
    /*
    SAME: ball color matches our alliance color
    DIFFERENT: ball is def opposing alliance ball
    NONE: there is no ball
    */

    // get opposite alliance color
    DriverStation.Alliance oppositeAlliance;
    
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      oppositeAlliance = DriverStation.Alliance.Blue;
    } else {
      oppositeAlliance = DriverStation.Alliance.Red;
    }

    if (isCorrectColor()) {
      return BallMatchesAlliance.SAME; 
    } else if (colorSensor.getColor() == oppositeAlliance) {
      return BallMatchesAlliance.DIFFERENT;
    } else if (colorSensor.getColor() == DriverStation.Alliance.Invalid) {
      return BallMatchesAlliance.NONE;
    }
    return null;
  }

  public boolean hasBall() {
    return !breakBeamSensor.get(); 
  }

  public enum BreakBeamDIO {
    // Represents DIO Port to initialize break beam sensor to
    FRONT(0),
    BACK(1);

    public final int dioPort;

    BreakBeamDIO(int dioPort) {
      this.dioPort = dioPort;
    }
  }

  public enum BallMatchesAlliance {
    SAME,
    DIFFERENT,
    NONE
  }

  public void setIsShooting(boolean isShooting) {
    this.isShooting = isShooting;
  }

  public boolean isShooting() {
    return isShooting;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    
    builder.setActuator(true); // Only allow setting values when in Test mode
    builder.setSafeState(() -> setSpeed(FeedSpeed.STOP)); // Provide method to make the subsystem safe

    if (hasColorSensor) {
      builder.addStringProperty("Color/Detected", () -> colorSensor.getDetectedColor().toString(), null);
      builder.addDoubleProperty("Color/Confidence", () -> colorSensor.getConfidence(), null);
      builder.addStringProperty("Color/ENUM", () -> colorSensor.getColor().toString(), null);
      builder.addStringProperty("Color/SameAllianceColor", () -> "" + isCorrectColor(), null);
      builder.addStringProperty("Color/whoseBall", () -> whoseBall().toString(), null);
      
      builder.addDoubleProperty("Color/RGB/Red", () -> colorSensor.getDetectedColor().red * 255, null);
      builder.addDoubleProperty("Color/RGB/Green", () -> colorSensor.getDetectedColor().green * 255, null);
      builder.addDoubleProperty("Color/RGB/Blue", () -> colorSensor.getDetectedColor().blue * 255, null);

    }
    builder.addBooleanProperty("hasBall", () -> hasBall(), null);
  }
}
