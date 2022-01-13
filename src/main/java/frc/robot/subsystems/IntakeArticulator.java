/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap.CAN;

import frc.robot.util.MercMotorController.*;
import frc.robot.util.interfaces.IMercMotorController;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;
import frc.robot.util.interfaces.IMercMotorController.LimitSwitchDirection;

public class IntakeArticulator extends SubsystemBase implements IMercShuffleBoardPublisher{

  private final IMercMotorController intakeArticulator;
  private IntakePosition intakePosition;
  private final double OUT_SPEED = 0.5, IN_SPEED = -0.6;

  /**
   * Creates a new IntakeArticulator.
   */
  public IntakeArticulator() {
    super();
    intakeArticulator = new MercTalonSRX(CAN.INTAKE_ARTICULATOR);
    intakeArticulator.setInverted(true);
    intakePosition = IntakePosition.IN;
  }

  public enum IntakePosition {
    OUT,
    IN
  }

  
  public void setIntakeIn() {
    this.intakePosition = IntakePosition.IN;
    intakeArticulator.setSpeed(IN_SPEED);
  }

  public void setIntakeOut() {
    this.intakePosition = IntakePosition.OUT;
    intakeArticulator.setSpeed(OUT_SPEED);
  }

  public IntakePosition getIntakePosition() {
    return this.intakePosition;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void publishValues() {
    SmartDashboard.putBoolean(getName() + "/FwdLimit", intakeArticulator.isLimitSwitchClosed(LimitSwitchDirection.FORWARD));
    SmartDashboard.putBoolean(getName() + "/RevLimit", intakeArticulator.isLimitSwitchClosed(LimitSwitchDirection.REVERSE));
    SmartDashboard.putNumber(getName() + "/ArticulateSpeed", intakeArticulator.getSpeed());
  }
}
