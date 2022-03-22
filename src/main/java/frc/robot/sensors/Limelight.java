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
    private LimelightLEDState ledState;
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
        ledState = LimelightLEDState.OFF;
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
            case "ledMode": {
                ledState = LimelightLEDState.valueOf(nv.getDouble());
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

    public double getDistanceToTarget(){
        //10.7 + -0.513x + 0.0128x^2

        //return (0.0128 * Math.pow(this.targetCenterYAngle, 2.0)) + (-0.513 * this.targetCenterYAngle) + 10.7;
        return (12.6 * Math.exp(0.0483 * this.targetCenterYAngle));
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
    public synchronized void setLEDState(LimelightLEDState ledState) {
        this.ledState = ledState;
        nt.getEntry("ledMode").setNumber(ledState.value);
    }

    public boolean getLEDState() {
        // We are assuming PIPELINE_DEFULT is ON here
        return ledState == LimelightLEDState.ON || ledState == LimelightLEDState.PIPELINE_DEFAULT;
    }

    public void switchLEDState() {
        setLEDState(getLEDState() ? LimelightLEDState.ON : LimelightLEDState.OFF);
    }

    public void setPipeline(int slot) {
        nt.getEntry("pipeline").setNumber(slot);
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
        public static LimelightLEDState valueOf(double value) {
            for (LimelightLEDState state : LimelightLEDState.values()) {
                if (state.value == value) return state;
            }
            return LimelightLEDState.OFF;
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