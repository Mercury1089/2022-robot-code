/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/* this is definently not a class to create a school shooter                  */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.CAN;
import frc.robot.sensors.Limelight;
import frc.robot.util.PIDGain;
import frc.robot.util.interfaces.IMercPIDTunable;

public class Shooter extends SubsystemBase implements IMercPIDTunable {

  public static final double NOMINAL_OUT = 0.0, PEAK_OUT = 1.0;
  public static final double MAX_RPM = 5000.0, STEADY_RPM = 3600.0, LOW_RPM = 1000.0, NULL_RPM = -1.0;
  public static final double MIN_DISTANCE = 6.7, MAX_DISTANCE = 17.0;
  //public static final double MIN_DISTANCE = 2.0, MAX_DISTANCE = 20.0;
  public final int BREAKBEAM_DIO = 2;
  private final double TARGET_VELOCITY_THRESHOLD = 50.0; // within a +- 50 rpm range to shoot
  private final double MAX_VOLTAGE = 10.5;

  private CANSparkMax shooterLeft, shooterRight;
  private double targetVelocity;
  private PIDGain velocityGains;
  private Limelight limelight;
  private DigitalInput breakBeamSensor;
  private boolean autoShootEnable;
  private double smartDashboardTargetVelocity = 0.0;
  private boolean useSpeed, setPID;
  private double smartdashkP = 0.00024, smartdashkF = 0.0002016, smartdashkI = 0.00000001, smartDashkD = 0.01;
  private int shootCount = 0;

  

  public enum ShooterMode {
    ONE_WHEEL, NONE
  }

  public Shooter(ShooterMode mode, Limelight limelight) {
    setName("Shooter");

    autoShootEnable = true;

    if (mode == ShooterMode.ONE_WHEEL) {
      shooterLeft = new CANSparkMax(CAN.SHOOTER_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
      shooterRight = new CANSparkMax(CAN.SHOOTER_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);

      shooterLeft.enableVoltageCompensation(MAX_VOLTAGE);
      shooterRight.enableVoltageCompensation(MAX_VOLTAGE);

      shooterLeft.getPIDController().setOutputRange(NOMINAL_OUT, PEAK_OUT);
      shooterRight.getPIDController().setOutputRange(NOMINAL_OUT, PEAK_OUT);

      shooterLeft.setIdleMode(IdleMode.kCoast);
      shooterRight.setIdleMode(IdleMode.kCoast);

      shooterLeft.setInverted(true);
      shooterRight.follow(shooterLeft, true); // Follow inverted
    } else if (mode == ShooterMode.NONE) {
      shooterLeft = shooterRight = null;
    }


    SmartDashboard.putNumber(getName() + "/SetRPM", 0.0);

    stopShooter();
    targetVelocity = 0.0;
    // velocityGains = new PIDGain(1e-5, 2e-7, 1e-5, 2.6e-4);
    velocityGains = new PIDGain(0.00024, 0.00000001, 0.01, 0.0002016);
    
    this.limelight = limelight;
  
    setPIDGain(SHOOTER_PID_SLOTS.VELOCITY_GAINS.getValue(), velocityGains);

    this.breakBeamSensor = new DigitalInput(BREAKBEAM_DIO); 
  }

  public void stopShooter() {
    targetVelocity = 0.0;
    shooterLeft.stopMotor();
  }

  /**
   * Get the current velocity of the shooter
   * @return the velocity in RPM
   */
  public double getVelocity() {
    return shooterLeft != null ? shooterLeft.getEncoder().getVelocity() : 0.0;
  }




  /**
   * Get the target velocity for the shooter based on a distance
   * using the distance to RPM equation.
   * @param distance distance to the target as provided by the limelight
   * @return calculated velocity based on distance
   */
  private double getVelocityFromDistance(double distance) {
    // return 75.0 + (2932.0 * Math.exp(0.0246 * distance));
   // return 3410 - 70.2 * distance + 7.71 * Math.pow(distance, 2) + 0.0349 * Math.pow(distance, 3);
    return 50 + 3772 - 165 * distance + 15.8 * Math.pow(distance, 2)  - 0.187 * Math.pow(distance, 3);
  }

  /**
   * Get the velocity required to shoot into the target
   * @return velocity required or NULL_RPM if velocity is invalid
   */
  public double getVelocityToTarget() {
    double distance = limelight.getDistanceToTarget();
    return distance >= MIN_DISTANCE && distance <= MAX_DISTANCE ? getVelocityFromDistance(distance) : NULL_RPM;
  }

  /**
   * Check if the shooter is running at the requested target velocity
   * @return true if at target velocity, false otherwise
   */
  public boolean isAtTargetVelocity() {
   // return Math.abs(getVelocity() - targetVelocity) <= 0.02 * targetVelocity;
      return Math.abs(getVelocity() - targetVelocity) <= TARGET_VELOCITY_THRESHOLD;
  }

  public boolean isReadyToShoot() {
    return this.autoShootEnable && isAtTargetVelocity() ;
  }

  public void setVelocity(double velocity) {
    if (shooterLeft != null && shooterRight != null)
    {
      // Record the target velocity for atTargetRPM()
      targetVelocity = velocity;
      // If the target velocity is outside the valid range, run at steady rate.
      double setVelocity = velocity != NULL_RPM && velocity <= MAX_RPM ? velocity : STEADY_RPM;
      shooterLeft.getPIDController().setReference(setVelocity, ControlType.kVelocity);
    }
  }

  public double getSmartDashboardRPM() {
    return SmartDashboard.getNumber(getName() + "/SetRPM", 0.0);
  }

  public boolean hasBall() {
    // if the shooter has a ball (if beam is broken)
    return !breakBeamSensor.get();
    
  }

  public WithinShooterBounds insideShooterBounds() {
    double distance = limelight.getDistanceToTarget();
    if (distance > MAX_DISTANCE) {
      return WithinShooterBounds.TOO_FAR;
    } else if (distance < MIN_DISTANCE) {
      return WithinShooterBounds.TOO_CLOSE;
    } else {
      return WithinShooterBounds.WITHIN_RANGE;
    }
  }

  public void setAutoShootEnable(boolean autoShoot) {
    this.autoShootEnable = autoShoot;
  }

  public boolean isAutoShootEnabled() {
    return this.autoShootEnable;
  }

  public void incrementShootCount() {
    this.shootCount += 1;
  }

  public int getShootCount() {
    return this.shootCount;
  }

  public double getTargetVelocity() {
    return smartDashboardTargetVelocity;
  }

  public void setTargetVelocity(double velocity) {
    smartDashboardTargetVelocity = velocity;
    
  }

  public void setSmartDashSpeed(boolean useSpeed) {
    this.useSpeed = useSpeed;
    if (useSpeed) {
      setVelocity(smartDashboardTargetVelocity);
    } 
  }

  public boolean getUseSpeed() {
    return this.useSpeed;
  }

  public void setPVal(double pval) {
    this.smartdashkP = pval;
  }

  public void setFVal(double kFVal) {
    this.smartdashkF = kFVal;
  }

  public void setDVal(double kDVal) {
    this.smartDashkD = kDVal;
  }

  public void setIVal(double kIVal) {
    this.smartdashkI = kIVal;
  }

  public double getPVal() {
    return this.smartdashkP;
  }

  public double getFVal() {
    return this.smartdashkF;
  }

  public double getDVal() {
    return this.smartDashkD;
  }

  public double getIVal() {
    return this.smartdashkI;
  }

  

  public void setSmartDashPID(boolean setPID) {
    this.setPID = setPID;
    if (this.setPID) {
      velocityGains = new PIDGain(smartdashkP, smartdashkI, smartDashkD, smartdashkF);
      setPIDGain(SHOOTER_PID_SLOTS.VELOCITY_GAINS.getValue(), velocityGains);
    } 
  }

  public boolean getSetPID() {
    return this.setPID;
  }



  @Override
  public void initSendable(SendableBuilder builder) {
    
    // builder.setActuator(true); // Only allow setting values when in Test mode
    builder.addBooleanProperty("ShooterHasBall", () -> hasBall(), null);
    builder.addDoubleProperty("CurrentRPM", () -> getVelocity(), null);
    builder.addDoubleProperty("TargetRPM", () -> targetVelocity, null);
    builder.addBooleanProperty("AtTargetRPM", () -> isAtTargetVelocity(), null);
    builder.addDoubleProperty("setTargetRPM", () -> getTargetVelocity(), (x) -> setTargetVelocity(x));
    builder.addBooleanProperty("setToTargetSpeed", () -> getUseSpeed(), (x) -> setSmartDashSpeed(x));

    builder.addDoubleProperty("SetPVal", () -> getPVal(), (x) -> setPVal(x));
    builder.addDoubleProperty("SetFVal", () -> getFVal(), (x) -> setFVal(x));
    builder.addDoubleProperty("SetIVal", () -> getIVal(), (x) -> setIVal(x));
    builder.addDoubleProperty("SetDVal", () -> getDVal(), (x) -> setDVal(x));
    builder.addBooleanProperty("SetPID", () -> getSetPID(), (x) -> setSmartDashPID(x));
    
    builder.addStringProperty("Within Target", () -> insideShooterBounds().toString(), null);
    builder.addDoubleProperty("Balls Shot", () -> getShootCount(), null);

    
  }

  @Override
  public PIDGain getPIDGain(int slot) {
    return this.velocityGains;
  }

  private void configPID(CANSparkMax sparkmax, int slot, PIDGain gains) {
    sparkmax.getPIDController().setP(gains.kP, slot);
    sparkmax.getPIDController().setI(gains.kI, slot);
    sparkmax.getPIDController().setD(gains.kD, slot);
    sparkmax.getPIDController().setFF(gains.kF, slot);
  }

  @Override
  public void setPIDGain(int slot, PIDGain gains) {
    this.velocityGains = gains;

    if (shooterLeft != null && shooterRight != null) {
      configPID(shooterLeft, SHOOTER_PID_SLOTS.VELOCITY_GAINS.getValue(), this.velocityGains);
      configPID(shooterLeft, SHOOTER_PID_SLOTS.VELOCITY_GAINS.getValue(), this.velocityGains);
    }
  }

  public enum WithinShooterBounds {
    WITHIN_RANGE,
    TOO_FAR,
    TOO_CLOSE
  }

  @Override
  public int[] getSlots() {
    return new int[] { 0 };
  }

  public enum SHOOTER_PID_SLOTS {
    VELOCITY_GAINS(0);

    private int value;

    SHOOTER_PID_SLOTS(int value) {
      this.value = value;
    }

    public int getValue() {
      return this.value;
    }

    
  }
}
