/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/**
 * Add your docs here.
 */
public class REVColor {

  private final I2C.Port i2cPort;
  private final ColorSensorV3 colorSensor;
  
  private Color detectedColor;

  public REVColor() {
    i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);
  }

  public Color getRawColor() {
    return colorSensor.getColor();
  }

  public Color getDetectedColor() {
    return detectedColor;
  }
}