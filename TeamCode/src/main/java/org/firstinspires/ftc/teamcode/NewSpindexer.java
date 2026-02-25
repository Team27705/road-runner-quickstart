package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.RTPTorctex;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

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

    private ElapsedTime bootkickerClock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private ElapsedTime bootkickerTimeOut = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean kickerIsRunning;

    private static boolean initalize;

    private boolean canSpin;
    private boolean isSpinning;

    private boolean bootkickerCalled;


    private enum SpindexerMode {
        Intake,
        Outtake
    }
    private enum SpindexerState {
        Ready,
        ChangeChamberClockwise,
        IsSpinning
    }

    private enum KickerState {
        SendKickerUp,
        SendKickerDown,
        Ready,
        WaitTillDown
    }

    private SpindexerMode spindexerMode;
    private SpindexerState spindexerState;
    private KickerState kickerState;

    private int[] intakePositions = {60,180,330};//index 0 is the degree to send slot 1 to intake, etc //150ish jumps each time might be lower
    private int[] outTakePositions = {270, 150, 30}; //index 0 is the degree to send slot 1 to outtake, etc
    private int currentChamber;
    public String x;

    public String posState;

    private int teleopMotif;
    private boolean colorFound;

    private long colorStartTime;

//    private Timing.Timer bootKickerTimer;

    private ElapsedTime bootKickerTimer = new ElapsedTime();


    private static final int TIME_TO_DETECT = 50;
    private static final int BOOTKICKER_DELAY = 2000;
    private int r,g,b, opacity;
    //Angle Rotations for intake:
    //Slot 1: 60?
    //slot 2; 120?
    //slot 3: 340?

    //Angle Rotations for Outtake/under flywheel
    //slot 1: 200 or 220??
    //slot 2:
    //slot 3: 30
    //https://docs.ftclib.org/ftclib/features/util#timing-functions replace timers with this

    //https://gm0.org/en/latest/docs/software/concepts/finite-state-machines.html
    //replace the ifs with a switch case
    public NewSpindexer (HardwareMap hardwareMap, boolean auto) {
        //Spindexer Servo
        crServo = hardwareMap.get(CRServo.class, "Spindexer Servo");
        analogEncoder = hardwareMap.get(AnalogInput.class, "Spindexer Encoder");
        spindexer = new RTPTorctex(crServo, analogEncoder);


        bootkicker = hardwareMap.get(Servo.class, "bootkicker");

        bootkicker.setDirection(Servo.Direction.REVERSE);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        if (auto) {
            inventory = new String[] {"P", "G", "P"};
            spindexerMode = SpindexerMode.Outtake;
        }
        else {
            spindexerMode = SpindexerMode.Intake;
        }

        bootKickerTimer = new Timing.Timer(BOOTKICKER_DELAY,TimeUnit.MILLISECONDS);
        initalize = false;
    }

    //**
    // Update class is for
    // */


    //https://github.com/NgDodo/FTC12209new/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Sorter.java#L17

//    public void update() {
//        //currently does not work servo doesnt respond
//
//        if (!initalize) {
//            bootkicker.setPosition(0);
//            bootkickerClock.reset();
//            bootkickerTimeOut.reset();
//            kickerIsRunning = false;
//            isSpinning = false;
//            initalize = true;
//        }
//
//
//
//        //update spindexer PID
//        if (kickerIsRunning || bootkickerClock.milliseconds() < 250)  {
//            canSpin = false;
//            x = "breaking";
//            return;
//        }
//        canSpin = true;
//        x = "not breaking";
//
//        spindexer.update();
//        if (Math.abs(spindexer.getPower()) > .1) {
//            isSpinning = true;
//            posState = "not at pos";
//            return;
//        }
//        posState = "at pos";
//
//        if (isSpinning) return;
//
//        updateColorSensor();
//
//        //no colors detected do not spin
//        //two modes, one for feeding the spindexer, one for sending it to outtake
//        //switch them with a boolean that gets updated from controller buttons or the auto sets the variables
//        if (spindexerMode == SpindexerMode.Intake ) {
//            if (!getColor().equals("E")) {
//                feedFromIntake();
//            }
//        }
//        else if (spindexerMode == SpindexerMode.Outtake) {
////            if () {
////
////            }
//        }
//        //set target?
////        if (bootkicker.)
//        //if color is detected, then set targetPosition of Spindexer
//    }

    public void update(Gamepad gamepad1) { //this is for teleop
        if (!initalize) { //ALWAYS BRING BOOTKICKER DOWN AFTER A RUN ALWAYS!!!!!
            bootkicker.setPosition(0);
            canSpin = true;
            spindexerMode = SpindexerMode.Intake;
            spindexer.setTargetRotation(intakePositions[0]);
            kickerState = KickerState.Ready;
            spindexerState =SpindexerState.Ready;

            currentChamber = 0;
        }

        spindexer.update();

        if (gamepad1.aWasReleased()) { // choose motif from button presses, have this add to telem
            switch (teleopMotif) {
                case 1:
                    motif = new String[] {"G","P","P"};
                    teleopMotif++;
                    break;
                case 2:
                    motif = new String[] {"P","G","P"};
                    teleopMotif++;
                    break;
                case 3:
                    motif = new String[] {"P","G","G"};
                    teleopMotif = 4;
                    break;
            }
        }

        if (gamepad1.xWasPressed()) {
            spindexerMode = SpindexerMode.Outtake;
        }
        if (gamepad1.backWasPressed()) {
            spindexerMode = SpindexerMode.Intake;
        }
        if (gamepad1.dpadUpWasReleased()) {
            //call fsm for servo
            kickerState = KickerState.SendKickerUp;
        }
        bootkickerFSM();

        if (!kickerState.equals(KickerState.Ready)) {

        }

        handleIndexingMode();
        //shooting modes
    }

    public void bootkickerFSM () {
        switch (kickerState) {
            case Ready:
                break;
            case SendKickerUp:
                canSpin = false;
                bootkicker.setPosition(0.45);
                kickerState = KickerState.SendKickerDown;
                bootKickerTimer.start();
                break;
            case SendKickerDown:
                if (bootKickerTimer.done()) {
                    bootkicker.setPosition(0);
                    kickerState = KickerState.SendKickerDown;
                    bootKickerTimer.start();
                }
                break;
            case WaitTillDown:
                if (bootKickerTimer.done()) {
                    canSpin = true;
                    kickerState = KickerState.Ready;
                }
                break;
        }
    }
    public void spindexerFSM () {
        switch (spindexerState) {
            case Ready:
                break;
            case ChangeChamberClockwise:
                if (canSpin) {
                    currentChamber++;
                    currentChamber = currentChamber % 3;//when changing from shooting manually set currentChamber to
                    spindexer.setTargetRotation(currentChamber);
                }
                break;
            case IsSpinning:
                if (spindexer.isAtTarget()) {
                    spindexerState = SpindexerState.Ready;
                }
                break;


        }
    }
    public void handleIndexingMode () {
        if (!canSpin || !spindexerState.equals(SpindexerState.Ready)) return;
        if (spindexerMode.equals(SpindexerMode.Outtake)) {
//            if () {
//
//            }
        }
        else if (spindexerMode.equals(SpindexerMode.Intake) && !isFull()) {
            updateColorSensor();

        }
    }


    //set this up as a finite state machine
    //first check which index/chamber position has the desired ball
    //then check if
//    public void shootingSequence () {
//        switch () {
//            case 0:
//            case 1:
//        }
//    }

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

//    public void feedFromIntake () {
////        if (isFull()) { //checks if full // may be unncessary
////            return;
////        }
//        for (int i = 0; i < 3; i++) {
//            if (inventory[i].equals("E")) {
//                spindexer.changeTargetRotation(intakePositions[i]);
//                currentChamber = i;
//                inventory[currentChamber] = getColor();
//                return;
//            }
//        }
//    }

    public void goToOuttake () {
        if (motif == null) motif = new String[] {"P", "G", "P"};
        for (int i = 0; i < 3; i++) {
            if (!checkInventory(i)) {

            }
        }

    }

    public boolean checkInventory (int targetColor) {
        for (int j = 0; j < 3; j++) {
            if (inventory[j].equals(motif[targetColor])) {
                spindexer.changeTargetRotation(outTakePositions[j]);
                return true;
            }
        }
        return false;
    }

    public void resetServo () {
        if (bootkicker.getPosition() != 0 && bootkickerTimeOut.milliseconds() >= 250 && isSpinning) {
            bootkicker.setPosition(0);
            bootkickerClock.reset();
            kickerIsRunning = false;
        }
    }

//    public void kick () {
//        if (bootkicker.getPosition() != .45 && bootkickerClock.milliseconds() >= 250 && !isSpinning) {
//            bootkicker.setPosition(.45);
//            bootkickerTimeOut.reset();
//            kickerIsRunning = true;
//        }
//
//
//    }

    public void updateColorSensor() {
//         NormalizedRGBA colors = colorSensor.getNormalizedColors();
//         RGB = new double[] {colors.red, colors.green, colors.blue};
        String colorDetected = detectArtifactColor();

        if (colorDetected.equals("E")) {
            colorFound = false;
            return;
        }

        if (!colorFound) {
            colorFound = true;
            colorStartTime = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - colorStartTime >= TIME_TO_DETECT) {
            if (inventory[currentChamber].equals("E")) {
                inventory[currentChamber] = colorDetected;
//                spindexerState = SpindexerState.
            }
            colorStartTime = 0;
            colorFound = true;
            //deactivate colorSensor
        }
    }

    public String detectArtifactColor () {
        r = colorSensor.red();
        g = colorSensor.green();
        b = colorSensor.blue();
        opacity = colorSensorARGB();
        if (opacity >= 200000000) {
            return "E";
        } else if (g > r && g > b && g > 50 && g < 600) {
            return "G";
        } else if (b > r && b > g && b > 50 && b < 600) { // for purple check
            return "P";
        }
        else return "E";
    }

    public int colorSensorARGB () {
        return colorSensor.argb();
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

    public void rotateSpindexerClockwise() {
    }


    @SuppressLint("DefaultLocale")
    public String log () {

        return "Red: " + r + "\nGreen: " + g + "\nBlue: " + b + "\nARGB: " + colorSensorARGB() + "\n Artifact Color: " + detectArtifactColor();
//        return String.format(
//                    "Red: %.10f\n"+
//                    "Green: %.10f\n"+
//                    "Blue: %.10f\n"+
//                            "ARGB: \n"+"Color Artifact: \n",
//                    r,
//                    g,
//                    b,
//                    colorSensorARGB(),
//                    detectArtifactColor()
//        );
    }

    @TeleOp(name = "Color Sensor Test", group = "test")
    public static class ColorSensorTest extends LinearOpMode{

        @Override
        public void runOpMode () {

            waitForStart();
            NewSpindexer newSpindexer = new NewSpindexer(this.hardwareMap, false);


            while (!isStopRequested()) {
                newSpindexer.update(gamepad1);

//                if (gamepad1.dpadDownWasReleased()) {
//                    newSpindexer.resetServo();
//                }
//                if (gamepad1.dpadUpWasReleased()){
//                    newSpindexer.kick();
//                }

                telemetry.addLine(newSpindexer.log());
                telemetry.addData("x: ",newSpindexer.x);
                telemetry.addData("Pos State: ", newSpindexer.posState);
                telemetry.addData("Spindexer Power", newSpindexer.spindexer.getPower());
                telemetry.update();
            }
        }
    }
}
