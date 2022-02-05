// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.CAN;
import frc.robot.sensors.Limelight;
import frc.robot.util.MercMath;

public class Turret extends SubsystemBase {
  
  private TalonSRX turret;
  private Limelight limelight;
  private static final int MAX_TURRET_RPM = 250;
  public static final double THRESHOLD_DEGREES = 1.0;
  private static double NORMAL_P_VAL = 0.11;
  private double positionInput;
  

  /** Creates a new Turret. */
  public Turret(Limelight limelight) {
    super();

    this.limelight = limelight;
    turret = new TalonSRX(CAN.TURRET);

    turret.configFactoryDefault();
    setName("Turret");

    turret.configNeutralDeadband(0.01);
    turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID.PRIMARY_PID_LOOP, RobotMap.CTRE_TIMEOUT);
    turret.setSensorPhase(false);
    turret.setInverted(false);

    turret.configNominalOutputForward(0, RobotMap.CTRE_TIMEOUT);
    turret.configNominalOutputReverse(0, RobotMap.CTRE_TIMEOUT);
    turret.configPeakOutputForward(1.0, RobotMap.CTRE_TIMEOUT);
    turret.configPeakOutputReverse(-1.0, RobotMap.CTRE_TIMEOUT);

    turret.configAllowableClosedloopError(RobotMap.PID.PRIMARY_PID_LOOP, 0, RobotMap.CTRE_TIMEOUT);

    turret.config_kP(RobotMap.PID.PRIMARY_PID_LOOP, NORMAL_P_VAL, RobotMap.CTRE_TIMEOUT);
    turret.config_kI(RobotMap.PID.PRIMARY_PID_LOOP, 0.0, RobotMap.CTRE_TIMEOUT);
    turret.config_kD(RobotMap.PID.PRIMARY_PID_LOOP, 0.0, RobotMap.CTRE_TIMEOUT);
    turret.config_kF(RobotMap.PID.PRIMARY_PID_LOOP, 0.0, RobotMap.CTRE_TIMEOUT);

    int absolutePosition = turret.getSensorCollection().getPulseWidthPosition();
    absolutePosition &= 0xFFF;

    turret.setSelectedSensorPosition(absolutePosition, RobotMap.PID.PRIMARY_PID_LOOP, RobotMap.CTRE_TIMEOUT);

  }

  public void setSpeed(Supplier<Double> speedSupplier) {
    turret.set(ControlMode.PercentOutput, speedSupplier.get());
  }

  public void setPosition(double pos) {
    // pos is in degrees
    double ticks = MercMath.degreesToEncoderTicks(pos);
    ticks *= 9; // 9:1 gear ratio
    turret.set(ControlMode.Position, ticks);
  }

  public double getAngleError() {
    return MercMath.encoderTicksToDegrees(turret.getClosedLoopError());
  }

  public boolean isOnTarget() {
    return (Math.abs(getAngleError()) < Turret.THRESHOLD_DEGREES);
  }

  public double getCustomTickInDegrees() {
    double ticks = turret.getSelectedSensorPosition(0) / 9;
    double degs = MercMath.encoderTicksToDegrees(ticks);
    return degs;
  }

  public void setPVal(double PVal) {
    NORMAL_P_VAL = PVal;
    turret.config_kP(RobotMap.PID.PRIMARY_PID_LOOP, PVal, RobotMap.CTRE_TIMEOUT);
  }

  public double getPVal() {
    return NORMAL_P_VAL;
  }

  public Limelight getLimelight() {
    return this.limelight;
  }

  public boolean isAligned(){
    return limelight.getTargetAcquired() && Math.abs(limelight.getTargetCenterXAngle()) <= THRESHOLD_DEGREES;
    //ON_TARGET_THRESHOLD_DEG;
  }

  public double getAngleToTarget(){
    return limelight.getTargetCenterXAngle();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    
    builder.setActuator(true); // Only allow setting values when in Test mode
    builder.addDoubleProperty("Encoder", () -> turret.getSelectedSensorPosition(0), null);
    builder.addDoubleProperty("Position",
                        () -> MercMath.encoderTicksToDegrees(turret.getClosedLoopTarget()/9),
                        (x) -> setPosition(x));
    builder.addDoubleProperty("EncoderDegrees", () -> getCustomTickInDegrees(), null);
    builder.addDoubleProperty("Velocity", () -> turret.getSelectedSensorVelocity(), null);
    builder.addDoubleProperty("PID/kP", () -> getPVal(), (x) -> setPVal(x));

    builder.addBooleanProperty("TargetAcquired", () -> getLimelight().getTargetAcquired(), null);
    builder.addBooleanProperty("LimelightLEDState", () -> getLimelight().getLEDState(), null);
    builder.addBooleanProperty("IsAligned", () -> isAligned(), null);

  }

    // Called repeatedly when this Command is scheduled to run


}
