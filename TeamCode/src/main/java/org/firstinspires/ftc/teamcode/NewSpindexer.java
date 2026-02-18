package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.RTPTorctex;

public class NewSpindexer {
    //include spindexer servo, light, booktkicker, and color sensor
    //note:deprecate Spindexer.java once done
    private RTPTorctex spindexer;
    private CRServo crServo;
    private AnalogInput analogEncoder;

    private Servo bootkicker;
    private RevColorSensorV3 colorSensor;
    private String[] inventory = {"E","","E"}; //E = empty, P = purple, G = green

    private String[] autoStartingInvetory

    private double[] RGB = {0,0,0};


    public NewSpindexer (HardwareMap hardwareMap, boolean auto) {
        //Spindexer Servo
        crServo = hardwareMap.get(CRServo.class, "Spindexer Servo");
        analogEncoder = hardwareMap.get(AnalogInput.class, "Spindexer Encoder");
        spindexer = new RTPTorctex(crServo, analogEncoder);


        bootkicker = hardwareMap.get(Servo.class, "bootkicker");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        if (auto) inventory = new String[] {"P", "G", "P"};


    }




    public void update () {
        //update spindexer PID
        spindexer.update();

//        if (!detectGreen() || !detectPurple) return; //no colors detected do nothing
//
//        if (bootkicker.)
        //if color is detected, then set targetPosition of Spindexer

    }

//    public bootkicker

    public boolean detectGreen () {
    return false;
    }

    public boolean detectPurple () {
    return false;
    }


    public void updateColorSensor() {
         NormalizedRGBA colors = colorSensor.getNormalizedColors();
         RGB = new double[] {colors.red, colors.green, colors.blue};
    }
    public double[] getColors() {
        return RGB;
    }

    public boolean isFull() {
        for (int i = 0; i < 3; i++) {
            if (inventory[i].equals("E")) {
                return false;
            }
        }
        return true;
    }


    @SuppressLint("DefaultLocale")
    public String log () {
        return String.format(
                    "Red: %.10f\n"+
                    "Green: %.10f\n"+
                    "Blue: %.10f\n",
                    RGB[0],
                    RGB[1],
                    RGB[2]
            );
        }

    @TeleOp(name = "Color Sensor Test", group = "test")
    public static class ColorSensorTest extends LinearOpMode{

        @Override
        public void runOpMode () {

            waitForStart();
            NewSpindexer newSpindexer = new NewSpindexer(this.hardwareMap);


            while (!isStopRequested()) {
                newSpindexer.update();
                telemetry.addLine(newSpindexer.log());

                telemetry.update();
            }
        }
    }
}
