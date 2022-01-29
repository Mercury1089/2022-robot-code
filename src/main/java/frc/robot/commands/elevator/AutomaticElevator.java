/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class AutomaticElevator extends CommandBase {

  private Elevator elevator;
  private ElevatorPosition targetPos;
  private boolean endable;

  /**
   * Creates a new GoToSetPosition.
   */
  public AutomaticElevator(Elevator elevator, ElevatorPosition targetPos, boolean endable) {
    addRequirements(elevator);
    setName("AutomaticElevator");
    this.elevator = elevator;
    this.targetPos = targetPos;
    this.endable = endable;
  }

  public AutomaticElevator(Elevator elevator, ElevatorPosition pos) {
    this(elevator, pos, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPosition(targetPos);
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
    return (endable && elevator.isInPosition());
  }
}
