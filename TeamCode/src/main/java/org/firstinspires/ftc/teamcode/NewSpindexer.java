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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.RTPTorctex;

public class NewSpindexer {
    //include spindexer servo, light, booktkicker, and color sensor
    //note:deprecate Spindexer.java once done
    private RTPTorctex spindexer;
    private CRServo crServo;
    private AnalogInput analogEncoder;

    private Servo bootkicker;
    private RevColorSensorV3 colorSensor;
    private String[] inventory = {"E","E","E"}; //E = empty, P = purple, G = green

    public static String[] motif;

    private double[] RGB = {0,0,0};

    private ElapsedTime bootkickerClock;

    private ElapsedTime bootkickerTimeOut;

    private boolean bootkickerDown;

    private boolean initalize;

    private enum Spindexermode {
        Intake,
        Outtake
    }

    private Spindexermode mode;

    //Angle Rotations for intake:
    //Slot 1: 60?
    //slot 2; 120?
    //slot 3: 340?

    //Angle Rotations for Outtake/under flywheel
    //slot 1: 200 or 220??
    //slot 2:



    public NewSpindexer (HardwareMap hardwareMap, boolean auto) {
        //Spindexer Servo
        crServo = hardwareMap.get(CRServo.class, "Spindexer Servo");
        analogEncoder = hardwareMap.get(AnalogInput.class, "Spindexer Encoder");
        spindexer = new RTPTorctex(crServo, analogEncoder);


        bootkicker = hardwareMap.get(Servo.class, "bootkicker");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        if (auto) {
            inventory = new String[] {"P", "G", "P"};
            mode = Spindexermode.Outtake;
        }
        else {
            mode = Spindexermode.Intake;
        }

        bootkickerClock.milliseconds();
        bootkickerTimeOut.milliseconds();
        initalize = false;
    }

    //**
    // Update class is for
    // */


    public void update() {
        //update spindexer PID
        spindexer.update();

        if (!kickerIsBlocking()) {
            resetKicker();
            return;
        }

        if (!spindexer.isAtTargetPos()) return;

        updateColorSensor();
        if (!detectGreen() || !detectPurple()) return; //no colors detected do not spin
        //two modes, one for feeding the spindexer, one for sending it to outtake
        //switch them with a boolean that gets updated from controller buttons or the auto sets the variables
        if (mode == Spindexermode.Intake ) {

        }
        else if (mode == Spindexermode.Outtake) {

        }
        //set target?
//        if (bootkicker.)
        //if color is detected, then set targetPosition of Spindexer

    }

    public void setMotif (int motifTagNum) { //may not be needed
        if (motifTagNum == 21) {
            motif = new String[] {"G","P","P"};
        }
        else if (motifTagNum == 22) {
            motif = new String[] {"P","G","P"};
        }
        else if (motifTagNum == 23) {
            motif = new String[] {"P","P","G"};
        }
    }

    public void feedFromIntake () {
//        if (isFull()) { //checks if full // may be unncessary
//            return;
//        }
        for (int i = 0; i < 3; i++) {
            if (!inventory[i].equals("E")) {

            }
        }
    }

    public void goToOuttake () {
        if (motif == null) motif = new String[] {"P", "G", "P"};
        for (int i = 0; i < 3; i++) {
            if (inventory[i].equals(motif)) {

            }
        }
    }

//    public void kick () {
//        bootkicker.setPosition(.75);
//        bootkickerTimeOut.reset();
//
//        resetKicker();
//    }
//
//    public boolean isDoneKicking () {
//        if (bootkickerTimeOut.milliseconds() >= 200) {
//            bootkickerDown = false;
//            resetKicker();
//            return true;
//        }
//        return false;
//    }
//    public void resetKicker() {
//        if (!bootkickerDown && isDoneKicking()) {
//            bootkicker.setPosition(0);
//            bootkickerClock.reset();
//        }
//        bootkickerDown = true;
//    }
//    public boolean kickerIsBlocking() {
//        return bootkickerDown && bootkickerClock.milliseconds() >= 200;
//    }

//    public bootkicker

    public boolean detectGreen () {
        if () {
            inventory[]
            return true;
        }
        return false;
    }

    public boolean detectPurple () {
        if () {
            inventory[]
            return true;
        }
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
            NewSpindexer newSpindexer = new NewSpindexer(this.hardwareMap, false);


            while (!isStopRequested()) {
                newSpindexer.update();
                telemetry.addLine(newSpindexer.log());

                telemetry.update();
            }
        }
    }
}
