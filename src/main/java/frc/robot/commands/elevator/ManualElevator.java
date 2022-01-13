/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.GAMEPAD_AXIS;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends CommandBase {
  /**
   * Creates a new ManualElevator.
   */
  private Elevator elevator;
  private final double DEADZONE = 0.2;

  public ManualElevator(Elevator elevator) {
    addRequirements(elevator);
    setName("ManualElevator");
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Robot.robotContainer.getGamepadAxis(GAMEPAD_AXIS.leftY);
    elevator.setSpeed(Math.abs(speed) < DEADZONE ? 0:speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
