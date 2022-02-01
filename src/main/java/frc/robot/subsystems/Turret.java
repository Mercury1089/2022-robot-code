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

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.CAN;
import frc.robot.util.MercMath;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

public class Turret extends SubsystemBase {
  
  private TalonSRX turret;
  private double runSpeed;
  private static final int MAX_ELEV_RPM = 18000;
  private static final double NORMAL_P_VAL = 0.1;
  private double positionInput;
  

  /** Creates a new Turret. */
  public Turret() {
    super();
    turret = new TalonSRX(CAN.TURRET);

    turret.configFactoryDefault();
    setName("Turret");

    runSpeed = 0.5;
    positionInput = 0.0;
    turret.setNeutralMode(NeutralMode.Brake);

    turret.configMotionAcceleration((int)(MercMath.revsPerMinuteToTicksPerTenth(18000 * 2)));
    turret.configMotionCruiseVelocity((int) MercMath.revsPerMinuteToTicksPerTenth(MAX_ELEV_RPM));

    turret.setSensorPhase(false);
    turret.setInverted(false);
    turret.configNominalOutputForward(0.05, RobotMap.CTRE_TIMEOUT);
    turret.configNominalOutputReverse(-0.05, RobotMap.CTRE_TIMEOUT);
    turret.configPeakOutputForward(1.0, RobotMap.CTRE_TIMEOUT);
    turret.configPeakOutputReverse(-1.0, RobotMap.CTRE_TIMEOUT);
    turret.configClosedLoopPeriod(0, 1);
    turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID.PRIMARY_PID_LOOP, RobotMap.CTRE_TIMEOUT);
    turret.configSetParameter(ParamEnum.eClearPositionOnLimitR, 1, 0, 0);
    turret.getSensorCollection().setQuadraturePosition(0, RobotMap.CTRE_TIMEOUT);
    
    turret.configAllowableClosedloopError(RobotMap.PID.PRIMARY_PID_LOOP, 4096 / 360, RobotMap.CTRE_TIMEOUT);
    turret.config_kP(RobotMap.PID.PRIMARY_PID_LOOP, NORMAL_P_VAL, RobotMap.CTRE_TIMEOUT);
    turret.config_kI(RobotMap.PID.PRIMARY_PID_LOOP, 0.0, RobotMap.CTRE_TIMEOUT);
    turret.config_kD(RobotMap.PID.PRIMARY_PID_LOOP, 0.0, RobotMap.CTRE_TIMEOUT);
    turret.config_kF(RobotMap.PID.PRIMARY_PID_LOOP, 0.0, RobotMap.CTRE_TIMEOUT);
    turret.configClosedLoopPeakOutput(RobotMap.PID.PRIMARY_PID_LOOP, MercMath.calculateFeedForward(MAX_ELEV_RPM), RobotMap.CTRE_TIMEOUT);
  }

  public void setSpeed(Supplier<Double> speedSupplier) {
    turret.set(ControlMode.PercentOutput, speedSupplier.get());
  }

  public void setPosition(double posSupplier) {
    positionInput = posSupplier;
    turret.set(ControlMode.Position, positionInput);
  }

  public double getPosInput() {
    return positionInput;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    
    builder.setActuator(true); // Only allow setting values when in Test mode
    builder.addDoubleProperty("Encoder/Value", () -> turret.getClosedLoopTarget(),  (x) -> setPosition(x));
    builder.addDoubleProperty("Encoder/Velocity", () -> turret.getSelectedSensorVelocity(), null);
  }

}