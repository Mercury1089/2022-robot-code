/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.REVColor.ControlPanelColor;
import frc.robot.subsystems.Spinner;

public class RotationControl extends CommandBase{
  
  private Spinner spinner;
  private ControlPanelColor sensorColor, currentColor, nextColor;

  private int colorsCrossed;

  private final int MINIMUM_COLORS_CROSSED = 25;
  private final int SLOWDOWN_RANGE = 2;

  public RotationControl(Spinner spinner) {
    addRequirements(spinner);
    setName("RotationControl");
    this.spinner = spinner;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spinner.setSpeed(spinner.getRunSpeed());
    colorsCrossed = 0;
    sensorColor = spinner.getColorSensor().get();
    currentColor = sensorColor;
    nextColor = spinner.getNextColor(currentColor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sensorColor = spinner.getColorSensor().get(); 
    //check if the color seen is the next color on the wheel   
    if(sensorColor == nextColor){
      currentColor = nextColor;
      colorsCrossed++;
      nextColor = spinner.getNextColor(currentColor);
    }
    //slow down the motor if the rotation is almost over
    if(colorsCrossed >= MINIMUM_COLORS_CROSSED - SLOWDOWN_RANGE
    && colorsCrossed < MINIMUM_COLORS_CROSSED)
      spinner.setSpeed(spinner.getRunSpeed() - spinner.getRunSpeed() / SLOWDOWN_RANGE);
    spinner.setColorsCrossed(colorsCrossed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spinner.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return colorsCrossed >= MINIMUM_COLORS_CROSSED;
  }
}
