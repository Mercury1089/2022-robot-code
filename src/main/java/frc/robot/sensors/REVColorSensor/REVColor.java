/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors.REVColorSensor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/**
 * Add your docs here.
 */
public class REVColor {

  
  private final ColorSensorV3 colorSensor;
  private final I2C.Port i2cPort;
  private final ColorMatch colorMatch;
  private final Color targetRed;
  private final Color targetBlue;
  private final double CONFIDENCE_THRESHOLD;
  private final double UPDATE_PERIOD_SECOND = 0.04;
  private double confidence;  
  private DriverStation.Alliance ballColor;
  private Notifier colorUpdater;
  
  
  private Color detectedColor = new Color(0.0, 0.0, 0.0);

  public REVColor() {

    ballColor = DriverStation.Alliance.Invalid;

    i2cPort = I2C.Port.kMXP;
    colorSensor = new ColorSensorV3(i2cPort);
    colorMatch = new ColorMatch();

    CONFIDENCE_THRESHOLD = 0.90;
    setConfidence(CONFIDENCE_THRESHOLD);

    // calibrated RGB's
    // targetRed = new Color(0.373, 0.451, 0.173);
    // targetBlue = new Color(0.263, 0.478, 0.255);

    targetRed = new Color(0.509, 0.373, 0.140);
    targetBlue = new Color(0.138, 0.407,0.458);

    colorMatch.addColorMatch(targetRed);
    colorMatch.addColorMatch(targetBlue);

    colorUpdater = new Notifier(() -> updateColor());
    colorUpdater.startPeriodic(UPDATE_PERIOD_SECOND);
  }

  private void updateColor() {

    detectedColor = colorSensor.getColor();
    try {
      ColorMatchResult match = colorMatch.matchColor(detectedColor);
      confidence = match.confidence;

      if (match.color == targetRed) {
        ballColor = DriverStation.Alliance.Red;
      } else if (match.color == targetBlue) {
        ballColor =  DriverStation.Alliance.Blue;
      } else {
        ballColor =  DriverStation.Alliance.Invalid;
      }
     

    } catch (Exception nullPointerException) {
      ballColor =  DriverStation.Alliance.Invalid;
    }
  }

  public DriverStation.Alliance getColor() {
    return ballColor;
  }

  public Color getDetectedColor() {
    return detectedColor;
  }

  public double getConfidence() {
    return confidence;
  }

  public void setConfidence(double confThresh) {
    colorMatch.setConfidenceThreshold(confThresh);
  }

}