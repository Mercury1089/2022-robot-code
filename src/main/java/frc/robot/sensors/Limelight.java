package frc.robot.sensors;

import edu.wpi.first.networktables.*;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.util.MercMath;

/**
 * A wrapper class for limelight information from the network table.
 * Example for commit
 */
public class Limelight {
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

    private IntegerSubscriber targetAcquiredSubscriber;
    private DoubleSubscriber targetCenterXAngleSubscriber, targetCenterYAngleSubscriber, targetAreaSubscriber;
    private IntegerSubscriber horizontalLengthSubscriber, verticalLengthSubscriber, shortLengthSubscriber, longLengthSubscriber;
    private IntegerEntry ledStateEntry;
    private IntegerPublisher pipelinePublisher;

    private final double areaCoeff = 16.2;
    private final double areaExp = -0.479;

    private LinearFilter movingAverage = LinearFilter.movingAverage(5);
    
    /**
     * Constucts the sensor and adds a listener to the table
     */
    public Limelight() {
        NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight-merc");

        targetAcquiredSubscriber = nt.getIntegerTopic("tv").subscribe(0);
        targetCenterXAngleSubscriber = nt.getDoubleTopic("tx").subscribe(0.0);
        targetCenterYAngleSubscriber = nt.getDoubleTopic("ty").subscribe(0.0);
        targetAreaSubscriber = nt.getDoubleTopic("ta").subscribe(0.0);
        horizontalLengthSubscriber = nt.getIntegerTopic("thor").subscribe(0);
        verticalLengthSubscriber = nt.getIntegerTopic("tvert").subscribe(0);
        shortLengthSubscriber = nt.getIntegerTopic("tshort").subscribe(0);
        longLengthSubscriber = nt.getIntegerTopic("tlong").subscribe(0);
        ledStateEntry = nt.getIntegerTopic("ledMode").getEntry(0);
        pipelinePublisher = nt.getIntegerTopic("pipeline").publish();
    }

    /**
     * u want the angle to target in x direction?
     *
     * @return angle to target in x direction
     */
    public double getTargetCenterXAngle() {
        return this.targetCenterXAngleSubscriber.get();
    }

    /**
     * u want the angle to target in y direction?
     *
     * @return angle to target in y direction
     */
    public double getTargetCenterYAngle() {
        return this.targetCenterYAngleSubscriber.get();
    }

    /**
     * u want the angle to target's area?
     *
     * @return angle to target in x direction
     */
    public double getTargetArea() {
        return this.targetAreaSubscriber.get();
    }

    /**
     * u want to know if the limelight sees a valid target?
     *
     * @return If there is a valid target
     */
    public boolean getTargetAcquired() {
        return this.targetAcquiredSubscriber.get() != 0;
    }

    public long getVerticalLength() {
        return this.verticalLengthSubscriber.get();
    }

    public synchronized long getHorizontalLength() {
        return this.horizontalLengthSubscriber.get();
    }

    public synchronized long getShortLength() {
        return this.shortLengthSubscriber.get();
    }

    public synchronized long getLongLenght() {
        return this.longLengthSubscriber.get();
    }

    /**
     * u want the distance based on the centerYAngle?
     *
     * @return the distance based on the centerYAngle
     */
    public double getDistanceToTarget(){
        //10.7 + -0.513x + 0.0128x^2

        //return (0.0128 * Math.pow(this.targetCenterYAngle, 2.0)) + (-0.513 * this.targetCenterYAngle) + 10.7;
        return (12.6 * Math.exp(0.0483 * this.targetCenterYAngleSubscriber.get()));
    }

    /**
     * u want the distance based on the area?
     *
     * @return the distance based on the area
     */
    public double getRawAreaDistance() {
        return areaCoeff * Math.pow(targetAreaSubscriber.get(), areaExp);
    }

    /**
     * u want the distance based on the vertical distance?
     *
     * @return the distance based on the vertical distance
     */
    public double getRawVertDistance() {
        return movingAverage.calculate(vertCoeff * Math.pow(getShortLength(), vertExp));
    }

    /**
     * u want the distance based on the horizontal distance?
     *
     * @return the distance based on the horizontal distance
     */
    public double getRawHorizDistance() {
        return horizCoeff * Math.pow(horizontalLengthSubscriber.get(), horizExp) * 12;
        //This is from 2019. Needs to be recalibrated for 2020
    }

    /**
     * Set the LED state on the limelight
     *
     * @param limelightLEDState the state of the LED.
     */
    public synchronized void setLEDState(LimelightLEDState ledState) {
        this.ledStateEntry.set(ledState.value);
    }

    public LimelightLEDState getLimelightLEDState() {
        return LimelightLEDState.valueOf(ledStateEntry.get());
    }
    public boolean getLEDState() {
        // We are assuming PIPELINE_DEFULT is ON here
        LimelightLEDState state = getLimelightLEDState();
        return state == LimelightLEDState.ON || state == LimelightLEDState.PIPELINE_DEFAULT;
    }

    public void switchLEDState() {
        setLEDState(getLEDState() ? LimelightLEDState.OFF : LimelightLEDState.ON);
    }

    public void setPipeline(int slot) {
        pipelinePublisher.set(slot);
    }

    public enum LimelightLEDState {
        ON(3), OFF(1), BLINKING(2), PIPELINE_DEFAULT(0);

        private long value;

        LimelightLEDState(long value) {
            this.value = value;
        }
    
        public static LimelightLEDState valueOf(long value) {
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