package org.firstinspires.ftc.teamcode.subsystems.hardwares;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Wrapper class for the RevColorSensorV3 to detect artifact colors (green, purple, or empty).
 * Provides methods to read color and opacity values and determine the detected color.
 */
public class ColorSensor {

    //----------------------------------------------------------------------------------------------
    // Constants
    //----------------------------------------------------------------------------------------------

    private static final int OPACITY_NO_OBJECT = 300000000;
    private static final int RGB_MIN = 50;
    private static final int RGB_MAX = 600;

    private final RevColorSensorV3 revColorSensor;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    /**
     * Constructs a ColorSensor using the provided hardware map.
     * @param hardwareMap the hardware map to retrieve the color sensor from
     */
    public ColorSensor(HardwareMap hardwareMap) {
        revColorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
    }

    //----------------------------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------------------------

    /**
     * Detects the color of the artifact in front of the sensor.
     * @return the detected color (GREEN, PURPLE, or EMPTY)
     */
    public DetectedColor detectArtifactColor() {
        int r = revColorSensor.red();
        int g = revColorSensor.green();
        int b = revColorSensor.blue();
        int opacity = getARGB();

        if (opacity >= OPACITY_NO_OBJECT) {
            // No object detected when opacity is above a certain threshold
            return DetectedColor.EMPTY;
        } else if (g > r && g > b && g > RGB_MIN && g < RGB_MAX) {
            // To check for green:
            // - G must be greater than R and B
            // - G must be above a minimum threshold to avoid false positives
            // - G must be below a maximum threshold to avoid false positives
            // Sometimes G or B will go to some absurd value in the hundreds.
            return DetectedColor.GREEN;
        } else if (b > r && b > g && b > RGB_MIN && b < RGB_MAX) {
            // To check for purple:
            // - B must be greater than R and G
            // - B must be above a minimum threshold to avoid false positives
            // - B must be below a maximum threshold to avoid false positives
            return DetectedColor.PURPLE;
        } else return DetectedColor.EMPTY;
    }

    /**
     * Gets the ARGB (opacity) value from the sensor.
     * @return the ARGB value
     */
    public int getARGB() {
        return revColorSensor.argb();
    }

    //----------------------------------------------------------------------------------------------
    // Enums
    //----------------------------------------------------------------------------------------------

    /**
     * Enum representing the possible detected colors.
     */
    public enum DetectedColor {
        /** No object or color detected. */
        EMPTY("E"),
        /** Green color detected. */
        GREEN("G"),
        /** Purple color detected. */
        PURPLE("P");

        /** Short code for the detected color. */
        public final String code;

        DetectedColor(String code) {
            this.code = code;
        }
    }
}