/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.RobotMap.CAN;
import frc.robot.util.MercMath;

public class Elevator extends SubsystemBase implements Sendable {

  private Relay elevatorLock;

  private static final int MAX_ELEV_RPM = 18000;
  private static final double ELEVATOR_POSITION_THRESHOLD = 500;
  private static final double NORMAL_P_VAL = 0.21;
  private static final int PRIMARY_PID_LOOP = 0;


  private TalonSRX elevator;

  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    super();
    setName("Elevator");

    elevatorLock = new Relay(RobotMap.RELAY.ELEVATOR_LOCK, Relay.Direction.kForward);
    elevatorLock.set(Relay.Value.kOff);

    elevator = new TalonSRX(CAN.ELEVATOR);
    elevator.configFactoryDefault();

    elevator.setNeutralMode(NeutralMode.Brake);

    elevator.configMotionAcceleration((int)(MercMath.revsPerMinuteToTicksPerTenth(18000 * 2)));
    elevator.configMotionCruiseVelocity((int) MercMath.revsPerMinuteToTicksPerTenth(MAX_ELEV_RPM));

    elevator.setSensorPhase(true);
    elevator.setInverted(true);
    elevator.configNominalOutputForward(0.125, RobotMap.CTRE_TIMEOUT);
    elevator.configNominalOutputReverse(-0.125, RobotMap.CTRE_TIMEOUT);
    elevator.configPeakOutputForward(1.0, RobotMap.CTRE_TIMEOUT);
    elevator.configPeakOutputReverse(-1.0, RobotMap.CTRE_TIMEOUT);
    elevator.configClosedLoopPeriod(0, 1);
    elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID.PRIMARY_PID_LOOP, RobotMap.CTRE_TIMEOUT);
    elevator.configSetParameter(ParamEnum.eClearPositionOnLimitR, 1, 0, 0);
    elevator.configForwardSoftLimitThreshold((int) ElevatorPosition.TOP.encPos, RobotMap.CTRE_TIMEOUT);
    elevator.configForwardSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT);
    elevator.getSensorCollection().setQuadraturePosition(0, RobotMap.CTRE_TIMEOUT);

    elevator.config_kP(Elevator.PRIMARY_PID_LOOP, NORMAL_P_VAL, RobotMap.CTRE_TIMEOUT);
    elevator.config_kI(Elevator.PRIMARY_PID_LOOP, 0.0, RobotMap.CTRE_TIMEOUT);
    elevator.config_kD(Elevator.PRIMARY_PID_LOOP, 0.0, RobotMap.CTRE_TIMEOUT);
    elevator.config_kF(Elevator.PRIMARY_PID_LOOP, 0.0, RobotMap.CTRE_TIMEOUT);
    elevator.configClosedLoopPeakOutput(Elevator.PRIMARY_PID_LOOP, MercMath.calculateFeedForward(MAX_ELEV_RPM), RobotMap.CTRE_TIMEOUT);

  }

  /**
   * Enable/disable the release on the ratchet that prevents upward movement of the elevator
   * @param state true to release the ratchet, false otherwise
   */
  public void setLockEngaged(boolean state){
    if (state) {
      elevatorLock.set(Relay.Value.kOn);
    } else {
      elevatorLock.set(Relay.Value.kOff);
    }
  }

  /**
   * Retrieve the status of the ratchet
   * @return true if the ratchet is released, false otherwise
   */
  public Relay.Value getLockState(){
    return elevatorLock.get();
  }

  /**
   * Move the elevator manually at a provided speed
   * @param speed the speed expressed as a double between -1.0 and 1.0
   */
  public void setSpeed(double speed){
    elevator.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Move the elevator to a specific position
   * @param pos the position to move to
   */
  public void setPosition(ElevatorPosition pos) {
    double targetEncPos = pos.encPos;
    if (pos.isRelative) {
      targetEncPos += getCurrentHeight();;
    }
    elevator.set(ControlMode.MotionMagic, targetEncPos);
  }

  /**
   * Check if the elevator at the last requested position
   * @return true if in position, false otherwise
   */
  public boolean isInPosition() {
    return (elevator.getClosedLoopError(PRIMARY_PID_LOOP) < ELEVATOR_POSITION_THRESHOLD ||
            elevator.getSensorCollection().isRevLimitSwitchClosed());
  }
  
  /**
   * Get current height of claw on elevator.
   * @return height of claw as read by the encoder, in ticks
   */
  public double getCurrentHeight() {
    return elevator.getSelectedSensorPosition(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    
    builder.setActuator(true); // Only allow setting values when in Test mode
    builder.setSafeState(() -> setSpeed(0)); // Provide method to make the subsystem safe
    builder.addDoubleProperty("Height", // property name for smartdash
                              () -> getCurrentHeight(), // getter function (returns current value)
                              (x) -> elevator.set(ControlMode.MotionMagic, x) // setter function (sets new value)
    );
    builder.addBooleanProperty("ForwardLimit", () -> elevator.getSensorCollection().isFwdLimitSwitchClosed(), null);
    builder.addBooleanProperty("ReverseLimit", () -> elevator.getSensorCollection().isRevLimitSwitchClosed(), null);
  }
  
  public enum ElevatorPosition{
    TOP(58000, false),       // Maximum height
    READY(43000, false),
    BOTTOM(-500, false),     // Negative value ensures we always move down until limit switch enabled
    HOOK(50000, false),      // Ready hook position
    HANG(-20000, true);    // Hang position - relative to current position.

    public final double encPos;
    public final boolean isRelative;

        /**
         * Creates an elevator position, storing the encoder ticks
         * representing the height that the elevator should be at.
         *
         * @param ep encoder position, in ticks
         */
        ElevatorPosition(double encPos, boolean isRelative) {
            this.encPos = encPos;
            this.isRelative = isRelative;
        }
  }
}
