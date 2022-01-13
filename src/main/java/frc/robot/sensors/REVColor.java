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
  private final double MINIMUM_CONFIDENCE_THRESHOLD;

  private final Color kBlueTarget;
  private final Color kGreenTarget;
  private final Color kRedTarget;
  private final Color kYellowTarget;
  
  private Color detectedColor;
  private double confidence = 0.0;

  public REVColor() {

    i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);
    colorMatch = new ColorMatch();
    /*
    //Without Light
    kBlueTarget = ColorMatch.makeColor(0.17, 0.475, 0.355);
    kGreenTarget = ColorMatch.makeColor(0.225, 0.575, 0.2);
    kRedTarget = ColorMatch.makeColor(0.63, 0.3, 0.07);
    kYellowTarget = ColorMatch.makeColor(0.4, 0.5, 0.1);
    */

    /*
    //With Light (need to fix)
    kBlueTarget = ColorMatch.makeColor(0.135, 0.435, 0.43);
    kGreenTarget = ColorMatch.makeColor(0.19, 0.56, 0.25);
    kRedTarget = ColorMatch.makeColor(0.49, 0.365, 0.145);
    kYellowTarget = ColorMatch.makeColor(0.32, 0.56, 0.12);
    */

    //With Light and Covering
    kBlueTarget = ColorMatch.makeColor(0.245, 0.465, 0.29);
    kGreenTarget = ColorMatch.makeColor(0.26, 0.49, 0.25);
    kRedTarget = ColorMatch.makeColor(0.35, 0.435, 0.215);
    kYellowTarget = ColorMatch.makeColor(0.315, 0.5, 0.185);
    

    colorMatch.addColorMatch(kBlueTarget);
    colorMatch.addColorMatch(kGreenTarget);
    colorMatch.addColorMatch(kRedTarget);
    colorMatch.addColorMatch(kYellowTarget); 
    
    MINIMUM_CONFIDENCE_THRESHOLD = 0.1;
    colorMatch.setConfidenceThreshold(MINIMUM_CONFIDENCE_THRESHOLD);

  }


  public ControlPanelColor get() {
    detectedColor = colorSensor.getColor();
    try {
      ColorMatchResult match = colorMatch.matchColor(detectedColor);
      confidence = match.confidence;

      if (match.color == kBlueTarget)
        return ControlPanelColor.BLUE;
      else if (match.color == kRedTarget)
        return ControlPanelColor.RED;
      else if (match.color == kGreenTarget)
        return ControlPanelColor.GREEN;
      else if (match.color == kYellowTarget) 
        return ControlPanelColor.YELLOW;
      return ControlPanelColor.UNKNOWN;  
    } catch (Exception nullPointerException) {
      confidence = -1;
      return ControlPanelColor.UNKNOWN;
    }
  }

  public double getConfidence() {
    return confidence;
  }

  public Color getRawColor() {
    return colorSensor.getColor();
  }

  public Color getDetectedColor() {
    return detectedColor;
  }

  public enum ControlPanelColor {
    BLUE,
    RED,
    GREEN,
    YELLOW,
    UNKNOWN
  }
}