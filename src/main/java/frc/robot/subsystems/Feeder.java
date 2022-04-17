/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sensors.REVColorSensor.PicoColorSensor;
import frc.robot.sensors.REVColorSensor.PicoREVColor;

public class Feeder extends SubsystemBase {
  
  public enum FeedSpeed{
    STOP(0.0), // stop running feeder motors
    LOAD(0.60), // run feeder motors inward
    EJECT(-1.00), // run feeder motors outward
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
  private PicoREVColor colorSensor;
  private DigitalInput breakBeamSensor;
  private int dioPort;
  private boolean isShooting;
  private boolean manualSmartDashShoot;

  /**
   * Creates a new Feeder.
   */
  public Feeder(PicoColorSensor picoColorSensor, int colorSensorID, BreakBeamDIO DIOPort, int motorControllerID) {

    dioPort = DIOPort.dioPort;

    
    breakBeamSensor = new DigitalInput(dioPort); 
    feedWheel = new VictorSPX(motorControllerID);
    feedWheel.configFactoryDefault();
    feedWheel.setInverted(false);
    feedWheel.setNeutralMode(NeutralMode.Brake);
    // No sensor feedback is required, so Status 2 frequency can be extra low.
    feedWheel.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, RobotMap.CAN_STATUS_FREQ.XTRA_LOW);
    setName("Feeder " + DIOPort.toString());
    this.colorSensor = new PicoREVColor(picoColorSensor);
    colorSensor.configID(colorSensorID);
    
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

  public boolean smartDashShoot() {
    return this.manualSmartDashShoot;
  }

  public void setSmartDashShoot(boolean setManual) {
    this.manualSmartDashShoot = setManual;
    if (manualSmartDashShoot) {
      setSpeed(FeedSpeed.SHOOT);
    } else {
      setSpeed(FeedSpeed.STOP);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    
    builder.setActuator(true); // Only allow setting values when in Test mode
    builder.setSafeState(() -> setSpeed(FeedSpeed.STOP)); // Provide method to make the subsystem safe

    builder.addDoubleProperty("Color/Confidence", () -> colorSensor.getConfidence(), null);
    builder.addStringProperty("Color/ENUM", () -> colorSensor.getColor().toString(), null);
    builder.addStringProperty("Color/SameAllianceColor", () -> "" + isCorrectColor(), null);
    builder.addStringProperty("Color/whoseBall", () -> whoseBall().toString(), null);
    
    builder.addDoubleProperty("Color/RGB/Red", () -> colorSensor.getDetectedColor().red * 255, null);
    builder.addDoubleProperty("Color/RGB/Green", () -> colorSensor.getDetectedColor().green * 255, null);
    builder.addDoubleProperty("Color/RGB/Blue", () -> colorSensor.getDetectedColor().blue * 255, null);

    builder.addDoubleProperty("Color/RGB/RawRed", () -> colorSensor.getDetectedColor().red, null);
    builder.addDoubleProperty("Color/RGB/RawGreen", () -> colorSensor.getDetectedColor().green, null);
    builder.addDoubleProperty("Color/RGB/RawBlue", () -> colorSensor.getDetectedColor().blue, null);

    builder.addBooleanProperty("Color/isConnected", () -> colorSensor.isSensorConnected(), null);
  

    builder.addBooleanProperty("ShootBall", () -> smartDashShoot(), (x) -> setSmartDashShoot(x));
    builder.addBooleanProperty("hasBall", () -> hasBall(), null);
  }
}
