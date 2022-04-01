// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors.REVColorSensor;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.sensors.REVColorSensor.PicoColorSensor.RawColor;

/** 
 * A version of REVColor using the Raspberry Pi Pico
 * to communicate with the ColorSensorV3
 * Created due to known I2C lockup issues
 * https://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues.html#onboard-i2c-causing-system-lockups
*/
public class PicoREVColor {
    private final ColorMatch colorMatch;
    private final double CONFIDENCE_THRESHOLD = 0.90;
    private final Color targetRed, targetBlue;
    private final PicoColorSensor picoColorSensor;

    private Color detectedColor = null;
    private RawColor detectedRawColor = new RawColor();
    private double confidence;
    private int index;

    public PicoREVColor(PicoColorSensor picoColorSensor) {


        this.picoColorSensor = picoColorSensor;
        colorMatch = new ColorMatch();
        setConfidence(CONFIDENCE_THRESHOLD);

        // calibrated RGB's
        targetRed = new Color(0.509, 0.373, 0.140);
        targetBlue = new Color(0.138, 0.407,0.458);

        colorMatch.addColorMatch(targetRed);
        colorMatch.addColorMatch(targetBlue); 
    }

    public void configID(int index) {
        this.index = index;
    }

        
    private Color convertRawColor() {
        // adjust raw color vals to 255 scale
        double r = (double) detectedRawColor.red;
        double g = (double) detectedRawColor.green;
        double b = (double) detectedRawColor.blue;
        double mag = r + g + b;
        return new Color(r / mag, g / mag, b / mag);
    }

    public DriverStation.Alliance getColor() {
        /*
        No notifier bc PicoColorSensor handles its own thread when talking to ColorSensorV3
        */
        picoColorSensor.getRawColor(detectedRawColor, index); // fill up RawColor obj
        
        detectedColor = convertRawColor();

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

    public boolean isSensorConnected() {
        return picoColorSensor.isSensorConnected(index);
    }

    public synchronized RawColor getRawColor() {
        picoColorSensor.getRawColor(detectedRawColor, index);
        return detectedRawColor;
    }

    public Color getDetectedColor() {
        if (detectedColor == null) {
            picoColorSensor.getRawColor(detectedRawColor, index);
            detectedColor = convertRawColor();
        }
        return detectedColor;
    }

    public double getConfidence() {
        return confidence;
      }

    public void setConfidence(double confThres) {
        colorMatch.setConfidenceThreshold(confThres);
    }
}
