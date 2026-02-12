package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Wrapper for the goBILDA® RGB Indicator Light (SKU 3118-0808-0002).
 * Operates on a PWM signal range of 500-2500μs.
 */
@SuppressWarnings("unused")
public class RGBIndicator {
    private final Servo servo;

    /**
     * goBILDA RGB Indicator Color Mappings
     */
    public enum Color {
        OFF(0.000),
        RED(0.279),
        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),
        GREEN(0.500),
        AZURE(0.555),
        BLUE(0.611),
        INDIGO(0.666),
        VIOLET(0.723),
        WHITE(1.000);

        public final double position;

        Color(double position) {
            this.position = position;
        }
    }

    /**
     * Private constructor to encourage use of the static .get() method.
     */
    private RGBIndicator(Servo servo) {
        this.servo = servo;
    }

    /**
     * Native-style accessor for the HardwareMap.
     * Use this like: RGBIndicator led = RGBIndicator.get(hardwareMap, "led");
     */
    public static RGBIndicator get(HardwareMap hwMap, String name) {
        return new RGBIndicator(hwMap.get(Servo.class, name));
    }

    /**
     * Set the color using a preset Enum
     */
    public void setColor(Color color) {
        servo.setPosition(color.position);
    }

    /**
     * Set a custom color using a raw position (0.0 to 1.0)
     */
    public void setColor() {
        setColor(0.0);
    }

    /**
     * Sets a custom color via position (0.0 to 1.0).
     */
    public void setColor(double position) {
        servo.setPosition(position);
    }

    // Helper methods for quick access
    public void setOff() { setColor(Color.OFF); }
    public void setRed() { setColor(Color.RED); }
    public void setOrange() { setColor(Color.ORANGE); }
    public void setYellow() { setColor(Color.YELLOW); }
    public void setSage() { setColor(Color.SAGE); }
    public void setGreen() { setColor(Color.GREEN); }
    public void setAzure() { setColor(Color.AZURE); }
    public void setBlue() { setColor(Color.BLUE); }
    public void setIndigo() { setColor(Color.INDIGO); }
    public void setViolet() { setColor(Color.VIOLET); }
    public void setWhite() { setColor(Color.WHITE); }

    /**
     * Provides access to the underlying Servo object if needed.
     */
    public Servo getInternalServo() {
        return servo;
    }
}