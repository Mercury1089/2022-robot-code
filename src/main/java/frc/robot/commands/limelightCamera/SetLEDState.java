/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.limelightCamera;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LimelightLEDState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightCamera;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetLEDState extends InstantCommand {

  private LimelightCamera limelightCamera;
  private LimelightLEDState state;

  public SetLEDState(LimelightCamera limelightCamera, LimelightLEDState state) {
    super();
    setName("SetLEDState");
    this.limelightCamera = limelightCamera;
    this.state = state;
    super.addRequirements(limelightCamera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelightCamera.getLimelight().setLEDState(state);
  }

  public boolean runsWhenDisabled(){
    return true;
  }
}
