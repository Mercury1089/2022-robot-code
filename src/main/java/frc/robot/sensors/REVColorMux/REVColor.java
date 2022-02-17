/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors.REVColorMux;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/**
 * Add your docs here.
 */
public class REVColor {

  
  private final ColorSensorV3 colorSensor;
  private final I2CMUX mux;
  private final ColorMatch colorMatch;
  private final Color targetRed;
  private final Color targetBlue;
  private final double CONFIDENCE_THRESHOLD;
  private double confidence;
  private final int colorSensorID;
  
  
  
  private Color detectedColor;

  public REVColor(ColorSensorID colorSensorIDEnum, I2CMUX mux) {
    this.mux = mux;

    colorSensorID = colorSensorIDEnum.id;
    
   
    synchronized(this) {
      mux.setEnabledBuses(colorSensorID);
      colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    }
    colorMatch = new ColorMatch();

    CONFIDENCE_THRESHOLD = 0.97;
    setConfidence(CONFIDENCE_THRESHOLD);

    // calibrated RGB's
    targetRed = new Color(0.373, 0.451, 0.173);
    targetBlue = new Color(0.263, 0.478, 0.255);

    colorMatch.addColorMatch(targetRed);
    colorMatch.addColorMatch(targetBlue);

    

  }

  public DriverStation.Alliance getColor() {
    detectedColor = getRawColor();
    try {
      ColorMatchResult match = colorMatch.matchColor(detectedColor);
      confidence = match.confidence;

      if (match.color == targetRed) {
        return DriverStation.Alliance.Red;
      } else if (match.color == targetBlue) {
        return DriverStation.Alliance.Blue;
      }
      return DriverStation.Alliance.Invalid;

    } catch (Exception nullPointerException) {
      return DriverStation.Alliance.Invalid;
    }
  }



  public synchronized Color getRawColor() {
    mux.setEnabledBuses(colorSensorID);
    return colorSensor.getColor();
  }

  public Color getDetectedColor() {
    if (detectedColor == null) {
      return getRawColor();
    }
    return detectedColor;
  }

  public double getConfidence() {
    return confidence;
  }

  public void setConfidence(double confThresh) {
    colorMatch.setConfidenceThreshold(confThresh); // need to change this?
  }




  public enum ColorSensorID {
    // enum to set correct enabled bus on MUX
    FRONT(0),
    BACK(1);

    public final int id; 

    ColorSensorID(int id) {
      this.id = id;
    }

  }
}