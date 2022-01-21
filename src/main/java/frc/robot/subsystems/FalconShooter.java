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
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.RobotMap.CAN;
import frc.robot.util.MercMath;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

public class FalconShooter extends SubsystemBase implements IMercShuffleBoardPublisher {

  private Relay elevatorLock;

  private static final int MAX_ELEV_RPM = 18000;
  private static final double NORMAL_P_VAL = 0.21;
  private static final int PRIMARY_PID_LOOP = 0;

  private TalonFX shooter_left, shooter_right;
  private double runSpeed;

  /**
   * Creates a new Elevator.
   */
  public FalconShooter() {
    super();
    setName("FalconShooter");

    shooter_left = new TalonFX(CAN.SHOOTER_LEFT);
    shooter_right = new TalonFX(CAN.SHOOTER_RIGHT);

    shooter_left.follow(shooter_right);

    runSpeed = 0.5;
  }

  public double getRunSpeed() {
    return runSpeed;
  }

  public void setSpeed(double speed){
    shooter_right.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void publishValues() {
    //SmartDashboard.putNumber(getName() + "/Height(ticks)", getEncTicks());
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
