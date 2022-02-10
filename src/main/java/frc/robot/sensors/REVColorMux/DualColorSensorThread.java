// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors.REVColorMux;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class DualColorSensorThread {
    private final TCA9548A MUX;
    private final Notifier NOTIFIER;
    private final int NUM_SENSORS = 2;
    private final double SAMPLE_PERIOD;

    // Set which bus on the chip each sensor is connected to, in this case
    // channels 1 and 2. Valid values are 0 - 7
    private final int[] sensorMuxIndex = {1, 2};
    private volatile SensorData[] sensorData = new SensorData[NUM_SENSORS];
    private final List<ColorSensorV3> SENSORS_LIST = new ArrayList<>();


    public DualColorSensorThread() {
        MUX = new TCA9548A();
        SAMPLE_PERIOD = 0.02;
        sensorData[0] = new SensorData();
        sensorData[1] = new SensorData();

        assert NUM_SENSORS < MUX.availableBuses();
        for (var idx : sensorMuxIndex) {
            assert idx < MUX.availableBuses();
        }

        for (int i = 0; i < NUM_SENSORS; i++) {
            MUX.setEnabledBuses(sensorMuxIndex[i]);
            var sensor = new ColorSensorV3(Port.kMXP);
            sensor.configureProximitySensor(
                ProximitySensorResolution.kProxRes8bit, ProximitySensorMeasurementRate.kProxRate12ms);
            sensor.configureColorSensor(
                ColorSensorResolution.kColorSensorRes13bit,
                ColorSensorMeasurementRate.kColorRate25ms,
                GainFactor.kGain9x);
            SENSORS_LIST.add(sensor);
            sensorData[i].color = new RawColor(0, 0, 0, 0);
            sensorData[i].distance = 0.0;
          }

        // Run this in a thread since I2C can be shady, and many transactions can be expensive
        // do we need notifier to query the ColorSensorV3?
        NOTIFIER = new Notifier(
                () -> {
                for (int i = 0; i < NUM_SENSORS; i++) {
                    MUX.setEnabledBuses(sensorMuxIndex[i]);
                    var sensor = SENSORS_LIST.get(i);
                    sensorData[i].color = sensor.getRawColor();
                    sensorData[i].distance = sensor.getProximity();
                }
                });

        NOTIFIER.startPeriodic(SAMPLE_PERIOD);
    }

    private class SensorData {
        public RawColor color;
        public double distance;
    }

    public ColorSensorV3 getColorSensor(int colorIndex) {
        if (colorIndex < NUM_SENSORS) {
            return SENSORS_LIST.get(colorIndex);
        }

        return null;

    }

}
