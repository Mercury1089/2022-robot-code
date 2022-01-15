/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class AutomaticElevator extends CommandBase {

  private Elevator elevator;
  private final double ELEVATOR_THRESHOLD = 500;
  private ElevatorPosition targetPos;
  private double targetEncPos;
  private boolean endable;

  /**
   * Creates a new GoToSetPosition.
   */
  public AutomaticElevator(Elevator elevator, ElevatorPosition pos, boolean endable) {
    addRequirements(elevator);
    setName("AutomaticElevator");
    this.elevator = elevator;
    targetPos = pos;
    targetEncPos = pos.encPos;
    this.endable = endable;
  }

  public AutomaticElevator(Elevator elevator, ElevatorPosition pos) {
    this(elevator, pos, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetEncPos = elevator.getCurrentHeight();
    if (targetPos.isRelative) {
      targetEncPos = targetEncPos + targetPos.encPos;
    } else {
      targetEncPos = targetPos.encPos;
    }
    elevator.getElevatorLeader().set(ControlMode.MotionMagic, targetEncPos);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endable && ELEVATOR_THRESHOLD >= Math.abs(targetEncPos - elevator.getCurrentHeight())) {
      return true;
    }
    if (targetPos == Elevator.ElevatorPosition.BOTTOM) {
      if (elevator.getElevatorLeader().getSensorCollection().isRevLimitSwitchClosed()) {
          elevator.getElevatorLeader().set(ControlMode.MotionMagic, targetEncPos);
          return true;
      }
    }
    return false;
  }
}
