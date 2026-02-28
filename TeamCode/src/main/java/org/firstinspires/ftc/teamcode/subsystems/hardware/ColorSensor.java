package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class ColorSensor {
    public static final int OPACITY_NO_OBJECT = 300000000;
    private static final int TIME_TO_DETECT = 25; //millis TODO: Increase delay and test
    public static final int RGB_MIN = 50;
    public static final int RGB_MAX = 600;
    private final RevColorSensorV3 revColorSensor;
    private final Spindexer spin;

    // State
    private boolean colorFound;
    private final ElapsedTime colorSensorTimer;
    private double colorStartTime;
    public boolean artifactDetected;
    private int r;
    private int g;
    private int b;

    public ColorSensor(RevColorSensorV3 revColorSensor, Spindexer spindexer) {
        this.revColorSensor = revColorSensor;
        this.spin = spindexer;
        colorSensorTimer = new ElapsedTime();
    }

    public void update() {
        //         NormalizedRGBA colors = colorSensor.getNormalizedColors();
        //         RGB = new double[] {colors.red, colors.green, colors.blue};

        String colorDetected = detectArtifactColor();

        if (colorDetected.equals("E")) {
            colorFound = false;
            artifactDetected = false;
            return;
        }

        if (!colorFound) {
            colorFound = true;
            colorSensorTimer.reset();
            colorStartTime = colorSensorTimer.milliseconds();
        }

        if (colorSensorTimer.milliseconds() - colorStartTime >= TIME_TO_DETECT) {
            if (spin.inventory[spin.currentChamber].equals("E")) {
                spin.inventory[spin.currentChamber] = colorDetected;
                artifactDetected = true;
            }
            colorStartTime = 0;
            colorFound = true;
            //deactivate colorSensor
        } else {
            artifactDetected = false;
        }
    }

    //TODO: RETUNE COLOR SENSOR
    //TODO: Optional get feedback from the GoBuilda light when updating/detecting a artifact
    public String detectArtifactColor() {
        r = revColorSensor.red(); //test if you dont need remove g> r and b > r and see if it
        // still works
        g = revColorSensor.green();
        b = revColorSensor.blue();
        int opacity = getARGB();
        if (opacity >= OPACITY_NO_OBJECT) { // higher opacity means no object detected
            return "E";
        } else if (g > r && g > b && g > RGB_MIN && g < RGB_MAX) { //sometimes g or b will go to
            // some absurd value in the hundreds
            return "G";
        } else if (b > r && b > g && b > RGB_MIN && b < RGB_MAX) { // for purple check
            return "P";
        } else return "E";
    }

    public int getARGB() {
        return revColorSensor.argb();
    }

    public boolean isArtifactDetected() {
        return artifactDetected;
    }

    public String log() {

        return "Red: " + r
                + "\nGreen: " + g
                + "\nBlue: " + b
                + "\nARGB: " + getARGB()
                + "\nArtifact Color: " + detectArtifactColor();
    }
}
