package frc.robot.sensors;

import edu.wpi.first.networktables.*;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.util.MercMath;

/**
 * A wrapper class for limelight information from the network table.
 * Example for commit
 */
public class Limelight implements TableEntryListener {
    private final double safeTurnThreshold = 10.0, limelightResX = 320;
    /*
     * Coefficients and exponents to help find the distance of a target Each
     * equation is in the form ax^b where a is the coefficient (Coeff) of the
     * equation and b is the exponent (Exp). x is the variable, either vertical
     * length (vert), horizontal length (horiz), or area (area).
     */
    // Redo Coefficients for current game
    private final double vertCoeff = 20428;
    private final double vertExp = -1.11;
    private final double horizCoeff = 264.0;
    private final double horizExp = -0.953;

    private NetworkTable nt; // finds the limelight network table
    private double numTargets, targetCenterXAngle, targetCenterYAngle, targetArea, horizontalLength, verticalLength, shortLength, longLength;
    private double[] cornerx;
    private boolean targetAcquired;
    private final double areaCoeff = 16.2;
    private final double areaExp = -0.479;

    private LinearFilter movingAverage = LinearFilter.movingAverage(5);
    
    /**
     * Constucts the sensor and adds a listener to the table
     */
    public Limelight() {
        nt = NetworkTableInstance.getDefault().getTable("limelight-merc");
        numTargets = 0.0;
        targetCenterXAngle = 0.0;
        targetCenterYAngle = 0.0;
        targetArea = 0.0;
        horizontalLength = 0.0;
        verticalLength = 0.0;
        targetAcquired = false;
        cornerx = new double[] {};
        nt.addEntryListener(this, EntryListenerFlags.kImmediate | EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        shortLength = 0.0;
        longLength = 0.0;
    }

    /**
     * @param nt    is always the limelight network table in this case
     * @param key   is the key of the entry that changed
     * @param ne    is the entry that changed
     * @param nv    is the value of the entry that changed
     * @param flags is the flag that occured which is always kUpdate in this case
     */
    @Override
    public void valueChanged(NetworkTable nt, String key, NetworkTableEntry ne, NetworkTableValue nv, int flags) {
        synchronized (this) {
            switch (key) {
            case "tx": {
                targetCenterXAngle = nv.getDouble();
                break;
            }
            case "ty": {
                targetCenterYAngle = nv.getDouble();
                break;
            }
            case "ta": {
                targetArea = nv.getDouble();
                break;
            }
            case "tv": {
                targetAcquired = nv.getDouble() != 0.0;
                break;
            }
            case "tl": {
                numTargets = nv.getDouble();
                break;
            }
            case "thor": {
                horizontalLength = nv.getDouble();
                break;
            }
            case "tvert": {
                verticalLength = nv.getDouble();
                break;
            }
            case "tcornx": {
                cornerx = nv.getDoubleArray();
            }
            case "tshort": {
                shortLength = nv.getDouble();
            }
            case "tlong": {
                longLength = nv.getDouble();
            }
            default: {
                break;
            }
            }
        }
    }

    /**
     * u want the angle to target in x direction?
     *
     * @return angle to target in x direction
     */
    public synchronized double getTargetCenterXAngle() {
        return this.targetCenterXAngle;
    }

    /**
     * u want the angle to target in y direction?
     *
     * @return angle to target in y direction
     */
    public synchronized double getTargetCenterYAngle() {
        return this.targetCenterYAngle;
    }

    /**
     * u want the angle to target's area?
     *
     * @return angle to target in x direction
     */
    public synchronized double getTargetArea() {
        return this.targetArea;
    }

    /**
     * u want the number of targets?
     *
     * @return number of visible targets
     */
    public synchronized double getNumTargets() {
        return this.numTargets;
    }

    /**
     * u want to know if the limelight sees a valid target?
     *
     * @return If there is a valid target
     */
    public synchronized boolean getTargetAcquired() {
        return this.targetAcquired;
    }

    /**
     * u want the vertical length of the target?
     *
     * @return the vertical length of the target
     */
    public synchronized double getVerticalLength() {
        return this.verticalLength;
    }

    /**
     * u want the number of targets?
     *
     * @return number of visible targets
     */
    public synchronized double getHorizontalLength() {
        return this.horizontalLength;
    }

    public synchronized double[] getCornerXArray() {
        return this.cornerx;
    }

    public synchronized double getShortLength() {
        return this.shortLength;
    }

    public synchronized double getLongLenght() {
        return this.longLength;
    }

    /**
     * u want the distance based on the area?
     *
     * @return the distance based on the area
     */
    public synchronized double getRawAreaDistance() {
        return calcDistFromArea();
    }

    /**
     * u want the distance based on the vertical distance?
     *
     * @return the distance based on the vertical distance
     */
    public synchronized double getRawVertDistance() {
        return calcDistFromVert();
    }

    /**
     * u want the distance based on the horizontal distance?
     *
     * @return the distance based on the horizontal distance
     */
    public synchronized double getRawHorizDistance() {
        return calcDistFromHoriz();
    }

    /**
     * Helper method for the area-dist calculation
     *
     * @return the distance based on area
     */
    public double calcDistFromArea() {
        return areaCoeff * Math.pow(targetArea, areaExp);
    }

    /**
     * Helper method for the vert-dist calculation
     *
     * @return the distance based on vertical distance
     */
    public double calcDistFromVert() {
        return movingAverage.calculate(vertCoeff * Math.pow(getShortLength(), vertExp));
    }

    /**
     * Helper method for the horiz-dist calculation
     *
     * @return the distance based on horizontal distance
     */
    public double calcDistFromHoriz() {
        return horizCoeff * Math.pow(horizontalLength, horizExp) * 12;
        //This is from 2019. Needs to be recalibrated for 2020
    }

    /**
     * Set the LED state on the limelight
     *
     * @param limelightLEDState the state of the LED.
     */
    public void setLEDState(LimelightLEDState limelightLEDState) {
        nt.getEntry("ledMode").setNumber(limelightLEDState.value);
    }

    public boolean getLEDState() {
        return nt.getEntry("ledMode").getNumber(0.0).doubleValue() == LimelightLEDState.ON.value ||
               nt.getEntry("ledMode").getNumber(0.0).doubleValue() == LimelightLEDState.PIPELINE_DEFAULT.value;
    }

    public void switchLEDState() {
        if ((double) (nt.getEntry("ledMode").getNumber(0.0)) == LimelightLEDState.OFF.getValue()) {
            setLEDState(LimelightLEDState.ON);
        } else {
            setLEDState(LimelightLEDState.OFF);
        }
    }

    public void setPipeline(int slot) {
        nt.getEntry("pipeline").setNumber(slot);
    }

    public synchronized boolean isSafeToTrack() {
        for (double corner : this.cornerx) {
            if (corner >= limelightResX - safeTurnThreshold || corner <= safeTurnThreshold) {
                System.out.println("not safe, turning!");
                return false;
            }
        }
        return true;
    }

    public enum LimelightLEDState {
        ON(3.0), OFF(1.0), BLINKING(2.0), PIPELINE_DEFAULT(0.0);

        private double value;

        LimelightLEDState(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    /**
     * Using the angle, it returns the distance between the target and the limelight
     * 
     * @return the distance based on vertical angle offset
     */

    public double calcDistFromAngle() {
        // return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_ANGLE +
        // getTargetCenterYAngle());
        /*
         * return MercMath.inchesToFeet(TARGET_LENGTH_INCHES) (VERTICAL_CAMERA_RES_PIXEL
         * / getVerticalLength()) / 2.0 /
         * Math.tan(MercMath.degreesToRadians(VFOV_DEGREES / 2));
         */
        return getVerticalLength();
    }
}