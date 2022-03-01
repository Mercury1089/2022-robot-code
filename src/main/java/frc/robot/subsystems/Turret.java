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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.CAN;
import frc.robot.sensors.Limelight;
import frc.robot.util.MercMath;

public class Turret extends SubsystemBase {
  
  private TalonSRX turret;
  private Limelight limelight;
  private static final int MAX_TURRET_RPM = 250;
  public static final double THRESHOLD_DEGREES = 3.0;
  private static double NORMAL_P_VAL = 0.11;
  private double positionInput;
  private double gearRatio = 9.0;

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

    turret.configForwardSoftLimitThreshold(4096*gearRatio, RobotMap.CTRE_TIMEOUT);
    turret.configReverseSoftLimitThreshold(0, RobotMap.CTRE_TIMEOUT);
    turret.configForwardSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT);
    turret.configReverseSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT);
   

    turret.configNominalOutputForward(0.02, RobotMap.CTRE_TIMEOUT);
    turret.configNominalOutputReverse(-0.02, RobotMap.CTRE_TIMEOUT);
    turret.configPeakOutputForward(0.25, RobotMap.CTRE_TIMEOUT);
    turret.configPeakOutputReverse(-0.25, RobotMap.CTRE_TIMEOUT);

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
    ticks *= gearRatio; // 9:1 gear ratio
    turret.set(ControlMode.Position, ticks);
  }

  public double getAngleError() {
    return MercMath.encoderTicksToDegrees(turret.getClosedLoopError()/gearRatio);
  }

  public boolean isOnTarget() {
    return (Math.abs(getAngleError()) < Turret.THRESHOLD_DEGREES);
  }

  public double getAngle() {
    return limelight.getTargetCenterXAngle();
  }

  public boolean isTargetAcquired(){
    return limelight.getTargetAcquired();
  }


  
  public double getCustomTickInDegrees() {
    double ticks = turret.getSelectedSensorPosition(0) / gearRatio;
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

  public boolean isAligned(){
    return Math.abs(limelight.getTargetCenterXAngle()) <= THRESHOLD_DEGREES;
    //ON_TARGET_THRESHOLD_DEG;
  }

  public boolean isReadyToShoot(){
    return isOnTarget();
}

  public boolean isAtForwardLimit(){
    return this.getCustomTickInDegrees() > 360;
  }

  public boolean isAtReverseLimit() {
    return this.getCustomTickInDegrees() < 0;
  }


  public double getAngleToTarget(){
    return limelight.getTargetCenterXAngle();
  }

  public double getDistanceToTarget() {
    return limelight.getDistanceToTarget();
  }

  public boolean targetIsLost() {
    // if the target is not aquired or it hits a limit
    return !isTargetAcquired() || (isAtForwardLimit() || isAtReverseLimit());
        // turretDefaultCommand.initialize()
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    
    builder.setActuator(true); // Only allow setting values when in Test mode
    builder.addDoubleProperty("Encoder", () -> turret.getSelectedSensorPosition(0), null);
    builder.addDoubleProperty("Position",
                        () -> MercMath.encoderTicksToDegrees(turret.getClosedLoopTarget()/gearRatio),
                        (x) -> setPosition(x));
    builder.addDoubleProperty("EncoderDegrees", () -> getCustomTickInDegrees(), null);
    builder.addDoubleProperty("DistanceToTarget", () -> limelight.getDistanceToTarget(), null);
    builder.addDoubleProperty("Velocity", () -> turret.getSelectedSensorVelocity(), null);
    builder.addDoubleProperty("PID/kP", () -> getPVal(), (x) -> setPVal(x));

    builder.addBooleanProperty("TargetAcquired", () -> limelight.getTargetAcquired(), null);
    builder.addBooleanProperty("LimelightLEDState", () -> limelight.getLEDState(), null);
    builder.addBooleanProperty("IsAligned", () -> isAligned(), null);
    builder.addDoubleProperty("LimelightXAngle", () -> getAngleToTarget(), null);
    builder.addBooleanProperty("isOnTarget", () -> isOnTarget(), null);
    builder.addStringProperty("hitForwardLimit", () -> isAtForwardLimit() + "", null);
    builder.addStringProperty("hitReverseLimit", () -> isAtReverseLimit() + "", null);

    //builder.addDoubleProperty(key, getter, setter);
    
  }

    // Called repeatedly when this Command is scheduled to run



}
