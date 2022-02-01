package frc.robot.util;

import frc.robot.RobotContainer;
import frc.robot.RobotMap.UNITS;
import frc.robot.subsystems.DriveTrain;
//import jdk.javadoc.internal.doclets.formats.html.resources.standard;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Class that contains various math functions.
 */
public class MercMath {
    private final static char[] HEX_ARRAY = "0123456789ABCDEF".toCharArray();

    /**
     * Clamps a value between a minimum and maximum, inclusive.
     *
     * @param val the value to clamp
     * @param min the minimum value
     * @param max the maximum value
     * @return {@code val}, if {@code val} is between [{@code min}, {@code max}]
     * {@code min}, if {@code val} is <= {@code min}
     * {@code min}, if {@code val} is >= {@code min}
     */
    public static double clamp(double val, double min, double max) {
        if (val <= min)
            val = min;
        else if (val >= max)
            val = max;

        return val;
    }
    /**
     * Clamps a value between a minimum and maximum, inclusive.
     *
     * @param val the value to clamp
     * @param min the minimum value
     * @param max the maximum value
     * @return {@code val}, if {@code val} is between [{@code min}, {@code max}]
     * {@code min}, if {@code val} is <= {@code min}
     * {@code min}, if {@code val} is >= {@code min}
     */
    public static int clamp(int val, int min, int max) {
        if (val <= min)
            val = min;
        else if (val >= max)
            val = max;

        return val;
    }
    
    // TODO: Make this work

    /**
     * Rounds a floating-point value to a certain number of places past the decimals
     *
     * @param val    the number to round
     * @param places the number of places to round to
     * @return the value, truncated to the setClawState amount of places
     */
    public static double roundFloat(double val, int places) {
        double intForm = val * Math.pow(10.0, places);
        int truncated = (int)intForm;
        truncated = truncated % 10 >= 5 ? truncated++ : truncated;
        return (double)truncated / Math.pow(10.0, places);
    }

    /**
     * Interpolates a value between a minimum and maximum value via a percentage
     *
     * @param percent percent to use to interpolate between a and b
     * @param a       some value
     * @param b       some other value
     * @return a value between a and b given a percent, with 0 being min, and 1 being max
     */
    public static double lerp(double percent, double a, double b) {
        percent = clamp(percent, 0, 1);

        return percent * b - percent * a + a;
    }

    /**
     * Does the law of cosines to find the third line length of a triangle
     * based off of the other two sides and the angle between those two
     *
     * @return the length of the third side, already square rooted
     */
    public static double lawOfCosines(double x, double y, double theta) {
        return Math.sqrt((x * x) + (y * y) - (2 * x * y * Math.cos(theta)));
    }

    /**
     * Does the law of sines to find an angle based off its opposite
     * side, and another angle and its opposite side
     *
     * @param x     a known sidelength
     * @param theta x's opposite angle  "Y DIS ! θ" - Aidan Sharpe
     * @param y     the side opposite to the angle you want to know
     * @return the unknown angle
     */
    public static double lawOfSinesAngle(double x, double y, double theta) {
        return Math.asin((y * Math.sin(theta)) / x);
    }

    public static double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0.0;
    }

    public static double applyDeadzone(double value) {
        return applyDeadzone(value, RobotContainer.DEADZONE);
    }

    public static double centimetersToInches(double val) {
        return val / 2.54;
    }

    public static double inchesToCentimeters(double val) {
        return val * 0.393700787;
    }

    public static double inchesToFeet(double inches) {
        return inches / 12.0;
    }

    public static double feetToInches(double feet) {
        return feet * 12.0;
    }

    public static double secondsToMinutes(double val) {
        return val / 60;
    }

    public static double minutesToSeconds(double val) {
        return val * 60;
    }

    public static double minutesToHours(double val) {
        return val / 60;
    }

    public static double hoursToMinutes(double val) {
        return val * 60;
    }

    public static int secondsToMilliseconds(double val) {
        return (int)Math.round(val * 1000);
    }

    public static double feetToMeters(double val) {
        return 100 * inchesToCentimeters(val / 12);
    }

    public static double metersToFeet(double val) {
        return 1 / (100 * inchesToCentimeters(val / 12));
    }

    public static double ticksToMeters(double ticks) {
        return inchesToCentimeters(encoderTicksToInches(ticks)) * 100;
    }

    public static double getEncPosition(double ticks) {
        return inchesToFeet((ticks / UNITS.MAG_ENCODER_TICKS_PER_REVOLUTION) *
                            (Math.PI * DriveTrain.WHEEL_DIAMETER_INCHES));
    }

    /**
     * Returns a value in ticks based on a certain value in feet using
     * the Magnetic Encoder.
     *
     * @param feet The value in feet
     * @return The value in ticks
     */
    public static double feetToEncoderTicks(double feet) {
        return inchesToEncoderTicks(feet * 12);
    }

    public static double inchesPerSecondToRevsPerMinute(double inchesPerSecond) {
        return (inchesPerSecond / (DriveTrain.WHEEL_DIAMETER_INCHES * Math.PI)) * 60;
    }

    public static double inchesPerSecondToTicksPerTenth(double inchesPerSecond) {
        return revsPerMinuteToTicksPerTenth(inchesPerSecondToRevsPerMinute(inchesPerSecond));
    }

    public static double inchesToEncoderTicks(double inches) {
        return inches / (Math.PI * DriveTrain.WHEEL_DIAMETER_INCHES) *
                (UNITS.MAG_ENCODER_TICKS_PER_REVOLUTION);
    }

    public static double encoderTicksToInches(double ticks) {
        return ticks / UNITS.MAG_ENCODER_TICKS_PER_REVOLUTION *
                (Math.PI * DriveTrain.WHEEL_DIAMETER_INCHES);
    }

    public static double degreesToPigeonUnits(double degrees) {
        return UNITS.PIGEON_NATIVE_UNITS_PER_ROTATION * degrees / 360;
    }

    public static double pigeonUnitsToDegrees(double pigeonUnits) {
        return pigeonUnits * 360 / UNITS.PIGEON_NATIVE_UNITS_PER_ROTATION;
    }

    public static double radiansToPigeonUnits(double radians) {
        return UNITS.PIGEON_NATIVE_UNITS_PER_ROTATION * Math.toDegrees(radians) / 360;
    }

    public static double radiansToDegrees(double rad) {
        return rad * 180.0 / Math.PI;
    }

    public static double degreesToRadians(double deg) {
        return deg * Math.PI / 180.0;
    }

    public static double encoderTicksToRevs(double ticks) {
        return ticks / UNITS.MAG_ENCODER_TICKS_PER_REVOLUTION;
    }

    public static double revsToEncoderTicks(double revs) {
        return revs * UNITS.MAG_ENCODER_TICKS_PER_REVOLUTION;
    }

    /**
     * <pre>
     *     public double ticksPerTenthToRevsPerMinute(double ticksPerTenthSecond)
     * </pre>
     * Returns value in revolutions per minute given ticks per tenth of a second.
     *
     * @param ticksPerTenthSecond
     * @return Revs per minute
     */
    public static double ticksPerTenthToRevsPerMinute(double ticksPerTenthSecond) {
        return ticksPerTenthSecond / UNITS.MAG_ENCODER_TICKS_PER_REVOLUTION * 600;
    }


    public static double revsPerMinuteToTicksPerTenth(double revsPerMinute) {
        return revsPerMinute / 600 * UNITS.MAG_ENCODER_TICKS_PER_REVOLUTION;
    }

    public static double revsPerMinuteToMetersPerSecond(double revsPerMinute) {
        return revsPerMinute * feetToMeters(DriveTrain.WHEEL_DIAMETER_INCHES * Math.PI / 12) / 60;
    }

    public static double ticksPerTenthToMetersPerSecond(double ticksPerTenth) {
        return revsPerMinuteToMetersPerSecond(ticksPerTenthToRevsPerMinute(ticksPerTenth));
    }

    public static double metersPerSecondToTicksPerTenth(double metersPerSecond) {
        return revsPerMinuteToTicksPerTenth(metersPerSecondToRevsPerMinute(metersPerSecond));
    }

    public static double metersPerSecondToRevsPerMinute(double metersPerSecond) {
        return (metersPerSecond / (inchesToMeters(DriveTrain.WHEEL_DIAMETER_INCHES) * Math.PI)) * 60;
    }

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }    

    public static double metersToEncoderTicks(double meters) {
        return inchesToEncoderTicks(metersToInches(meters));
    }

    public static double metersToInches(double meters) {
        return meters / 0.0254;
    }

    public static double calculateFeedForward(double revsPerMinute) {
        final double MAX_MOTOR_OUTPUT = 1023;
        final double NATIVE_UNITS_PER_100 = revsPerMinuteToTicksPerTenth(revsPerMinute);
        return MAX_MOTOR_OUTPUT / NATIVE_UNITS_PER_100;
    }

    public static String bytesToHex(byte[] bytes) {
        char[] hexChars = new char[bytes.length * 2];
        for (int j = 0; j < bytes.length; j++) {
            int v = bytes[j] & 0xFF;
            hexChars[j * 2] = HEX_ARRAY[v >>> 4];
            hexChars[j * 2 + 1] = HEX_ARRAY[v & 0x0F];
        }
        return new String(hexChars);
    }

    // This doesn't work yet. At least that's what I recall.
    public static String intsToHex(int[] ints) {
        StringBuilder hexString = new StringBuilder();
        for (int i : ints)
            hexString.append(Integer.toHexString(i));

        return hexString.toString();
    }

    // Some byte math conversion thing
    public static String bbToString(ByteBuffer bb) {
        final byte[] b = new byte[bb.remaining()];
        bb.duplicate().get(b);
        bb.rewind();
        return MercMath.bytesToHex(b);
    }

    public static double distanceFormula(double x1, double x2, double y1, double y2) {
        double deltaX = x2 - x1,
               deltaY = y2 - y1;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public static double distanceFormula(double deltaX, double deltaY) {
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public static double pose2dToDistance(Pose2d current, Pose2d previous) {
        Translation2d delta = current.relativeTo(previous).getTranslation();
        return distanceFormula(delta.getX(), delta.getY());
    }

    public static double translation2dToDistance(Translation2d current, Translation2d previous) {
        return current.getDistance(previous);
    }
}
