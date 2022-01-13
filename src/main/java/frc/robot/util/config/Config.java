package frc.robot.util.config;

import java.io.FileReader;
import java.util.Properties;

/**
 * Abstract wrapper class for config files.
 * This needs to be initialized prior to usage.
 * This should not be instantiated, rather other classes need to implement it.
 */
@Deprecated
public abstract class Config {
    private static Properties instance;

    /**
     * Appends the loaded properties file into this instance
     *
     * @param fileName the properties file to load
     */
    protected static void loadProperties(String fileName) {
        Properties temp = new Properties();
        if (instance != null) {
            try {
                FileReader loader = new FileReader("/home/lvuser/" + fileName);
                temp.load(loader);
                instance.putAll(temp);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    protected static Properties getInstance() {
        if (instance == null)
            instance = new Properties();
        return instance;
    }

    /**
     * Parses a property that is written as an array
     *
     * @param key       the property key to get the value from
     * @param delimiter the string separating the elements of the array
     * @return String array of the property's value,
     * or a string array with only "" if there is nothing.
     */
    protected static String[] parseArrayValue(String key, String delimiter) {
        String[] arr = instance.getProperty(key, "").split(delimiter);

        if (arr.length == 0)
            return new String[]{""};

        for (int i = 0; i < arr.length; i++)
            arr[i] = arr[i].trim();

        return arr;
    }
}