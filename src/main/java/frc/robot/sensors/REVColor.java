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
  private final ColorMatch colorMatch;
  private final Color targetRed;
  private final Color targetBlue;
  private final double CONFIDENCE_THRESHOLD;
  private double confidence;
  
  
  
  private Color detectedColor;

  public REVColor(ColorSensorPort port) {


    if (port == ColorSensorPort.FRONT_SENSOR) {
      i2cPort = I2C.Port.kOnboard;
    } else if (port == ColorSensorPort.BACK_SENSOR) {
      i2cPort = I2C.Port.kMXP;
    } else {
      i2cPort = null;
    }
    
    colorSensor = new ColorSensorV3(i2cPort);
    colorMatch = new ColorMatch();

    CONFIDENCE_THRESHOLD = 0.9;
    setConfidence(CONFIDENCE_THRESHOLD);
    // raw RGB vals
   // targetRed = new Color(1.0, 0.0, 0.0);
    // targetBlue = new Color(0.0, 0.0, 1.0);

    // calibrated RGB's
    targetRed = new Color(0.471, 0.376, 0.149);
    //targetBlue = new Color(0.204, 0.427, 0.364);
    targetBlue = new Color(0.16, 0.408, 0.424);

    colorMatch.addColorMatch(targetRed);
    colorMatch.addColorMatch(targetBlue);

    // move this out of setConfidence()
    // colorMatch.setConfidenceThreshold(confidenceThreshold); // need to change this?

  }

  public CargoColor getColor() {
    detectedColor = colorSensor.getColor();
    try {
      ColorMatchResult match = colorMatch.matchColor(detectedColor);
      confidence = match.confidence;

      if (match.color == targetRed) {
        return CargoColor.RED;
      } else if (match.color == targetBlue) {
        return CargoColor.BLUE;
      }
      return CargoColor.UNKNOWN;

    } catch (Exception nullPointerException) {
      return CargoColor.UNKNOWN;
    }
  }



  public Color getRawColor() {
    return colorSensor.getColor();
  }

  public Color getDetectedColor() {
    if (detectedColor == null) {
      return colorSensor.getColor();
    }
    return detectedColor;
  }

  public double getConfidence() {
    return confidence;
  }

  public void setConfidence(double confThresh) {
    colorMatch.setConfidenceThreshold(confThresh); // need to change this?
  }


  public enum CargoColor {
    RED,
    BLUE,
    UNKNOWN
  }

  public enum ColorSensorPort {
    FRONT_SENSOR,
    BACK_SENSOR
  }
}