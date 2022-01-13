package frc.robot.util.config;

import java.util.Properties;

/**
 * Class that allows user to interface with drivetrain.properties
 * config file with easy-to-access methods.
 * <p>
 * Note that this class should NOT be modified; if settings need to be
 * modified, edit the drivetrain.properties file.
 */
@Deprecated
public class DriveTrainSettings extends Config {

    private static Properties instance;

    public static void initialize() {
        instance = getInstance();
        loadProperties("drivetrain.properties");
    }

    /**
     * Gets the controller layout used for the drive train.
     *
     * @return the drive train layout
     */
    public static DriveTrainLayout getControllerLayout() {
        String layout = instance.getProperty("driveTrain.layout", "default").trim();

        return DriveTrainLayout.valueOf(layout.toUpperCase());
    }

    /**
     * Gets the gear ratio of the drive train's gearbox
     *
     * @return double representing the calculated gear ratio
     */
    public static double getGearRatio() {
        String[] nums = parseArrayValue("driveTrain.gearRatio", ":");
        double driver = 1.0, driven = 1.0;

        if (nums.length == 2) {
            driver = Double.parseDouble(nums[0].trim());
            driven = Double.parseDouble(nums[1].trim());
        }


        return driver / driven;
    }

    /**
     * Gets the diameter of the wheels on the drive train
     *
     * @return wheel diameter, in inches
     */
    public static double getWheelDiameter() {
        String val = instance.getProperty("driveTrain.wheelDiameter", "5.125").trim();

        return Double.parseDouble(val);
    }

    /**
     * Gets the max RPM of the drive train
     *
     * @return max RPM of drive train
     */
    public static double getMaxRPM() {
        String val = instance.getProperty("driveTrain.maxRPM", "700.63").trim();

        return Double.parseDouble(val);
    }

    /**
     * Gets the max output for the drive train motor controllers
     *
     * @return max output, represented as percent value [0.0, 1.0]
     */
    public static double getMaxOutput() {
        String val = instance.getProperty("driveTrain.maxOutput", "1.0").trim();

        return Double.parseDouble(val);
    }

    /**
     * Gets the minimum percent VBus value for DegreeRotate PID
     *
     * @return min output, represented as percent value [0.0, 1.0]
     */
    public static double getRotMinPVBus() {
        String val = instance.getProperty("rotateRelative.minPercentVBus", "0.3").trim();

        return Double.parseDouble(val);
    }

    /**
     * Gets the min and max value for any DriveTrain command's PID
     *
     * @param cmd the command to get the output range for
     * @return double array containing minimum and maximum output PID range
     */
    public static double[] getOutputRange(String cmd) {
        String[] arr = parseArrayValue(cmd + ".outputRange", ",");
        double[] range = {-0.5, 0.5};

        if (arr.length == 2) {
            range[0] = Double.parseDouble(arr[0]);
            range[1] = Double.parseDouble(arr[1]);
        }

        return range;
    }

    /**
     * Gets the absolute tolerance for DegreeRotate PID
     *
     * @return max degree delta from target heading
     */
    public static double getRotAbsTolerance() {
        String val = instance.getProperty("rotateRelative.absoluteTolerance", "1").trim();

        return Double.parseDouble(val);
    }

    /**
     * Gets the PID values for DegreeRotate
     *
     * @return double array containing PID values
     */
    public static double[] getPIDValues(String cmd) {
        String[] arr = parseArrayValue(cmd + ".PID", ",");
        double[] pid = {0.005, 0, 0.000};

        if (arr.length == 3) {
            pid[0] = Double.parseDouble(arr[0]);
            pid[1] = Double.parseDouble(arr[1]);
            pid[2] = Double.parseDouble(arr[2]);
        }

        return pid;
    }

    /**
     * Enumeration of drivetrain layouts.
     * <p>
     * {@code DEFAULT}: 2 Victor SPX followers, 2 Talon SRX leaders
     * <p>
     * {@code LEGACY}: 2 Talon SRX followers, 2 Talon SRX leaders
     */
    public enum DriveTrainLayout {
        SPARKS,
        TALONS,
        LEGACY
    }
}
