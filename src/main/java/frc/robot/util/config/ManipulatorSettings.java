package frc.robot.util.config;


/**
 * Class that allows user to interface with manipulator.properties
 * config file with easy-to-access methods.
 * <p>
 * Note that this class should NOT be modified; if settings need to be
 * modified, edit the manipulator.properties file.
 */
@Deprecated
public class ManipulatorSettings extends Config {


    public static void initialize() {
        loadProperties("manipulator.properties");
    }

    /**
     * Gets the PID values for the elevator
     *
     * @return double array containing PID values
     */
    public static double[] getElevatorPID() {
        String[] vals = parseArrayValue("elevator.PID", ",");
        double[] pid = {0.1, 0, 0};

        if (vals.length == 3) {
            pid[0] = Double.parseDouble(vals[0]);
            pid[1] = Double.parseDouble(vals[1]);
            pid[2] = Double.parseDouble(vals[2]);
        }

        return pid;
    }
}
