package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.RTPTorctex;

public class NewSpindexer {
    //include spindexer servo, light, booktkicker, and color sensor
    //note:deprecate Spindexer.java once done

    // Static constants
    private static final int TIME_TO_DETECT = 25; //millis TODO: Increase delay and test
    private static final int BOOTKICKER_DELAY = 400; //millis
    public static String[] motif;
    private static boolean isInitalized;

    public String x;
    public String posState;

    // Hardware
    private final RTPTorctex sorter;
    private final Servo bootkicker;
    private final ColorSensor colorSensor;
    // Bot Variables
    private String[] inventory = {"E", "E", "E"}; //E = empty, P = purple, G = green

    // State Variables
    private boolean canSpin;
    private boolean bootkickerCalled;
    private SpindexerMode spindexerMode;
    private SorterState sorterState;
    private KickerState kickerState;
    private ShootSequenceState shootSequenceState;
    private final int[] outTakePositions = {190, 310, 70}; //index 0 is the degree to send slot 1 to intake, etc
    private final int[] intakePositions = {10, 130, 250}; //index 0 is the degree to send slot 1 to outtake, etc
    private int currentChamber;
    private int teleopMotif;
    private boolean colorFound;
    private double colorStartTime;
    private final ElapsedTime bootKickerTimer;
    private final ElapsedTime sorterTimer;
    private final ElapsedTime colorSensorTimer;
    private final ElapsedTime shootSequenceTimer;

    private int currentTargetMotifNum = 0;
    public String flagActive;

    private int r, g, b, opacity;


    //https://gm0.org/en/latest/docs/software/concepts/finite-state-machines.html
    //replace the ifs with a switch case
    public NewSpindexer(HardwareMap hardwareMap, boolean auto) {
        //Spindexer Servo
        CRServo crServo = hardwareMap.get(CRServo.class, "Spindexer Servo");
        AnalogInput analogEncoder = hardwareMap.get(AnalogInput.class, "Spindexer Encoder");
        sorter = new RTPTorctex(crServo, analogEncoder);
        RevColorSensorV3 revColorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor = new ColorSensor(revColorSensor);

        bootkicker = hardwareMap.get(Servo.class, "bootkicker");
        bootkicker.setDirection(Servo.Direction.REVERSE);

        if (auto) {
            inventory = new String[] {"P", "G", "P"}; //Todo: physically label chambers 1, 2, 3
            spindexerMode = SpindexerMode.Outtake;
        }
        else {
            spindexerMode = SpindexerMode.Intake;
        }

        bootKickerTimer = new ElapsedTime();
        sorterTimer = new ElapsedTime();
        colorSensorTimer = new ElapsedTime();
        shootSequenceTimer = new ElapsedTime();
        isInitalized = false;
    }

    //https://github.com/NgDodo/FTC12209new/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Sorter.java#L17

    //**
    // This update class is for auton
    // *//

    public void update() {
        if (!isInitalized) {
            bootkicker.setPosition(0);
            canSpin = true;

            spindexerMode = SpindexerMode.Outtake;

            kickerState = KickerState.Ready;
            sorterState = SorterState.Ready;

            isInitalized = true;
//            currentChamber = 0;
            return;
        }

        //only let the Torctex PID update if the kicker is down. May have to forcibly set power in a else {.setPower(0);}
        if (kickerState.equals(KickerState.Ready)) {
            sorter.update();
        }

        if (spindexerMode.equals(SpindexerMode.Outtake)) {
            AutoShootingSequenceAuton();
        }
        else if (spindexerMode.equals(SpindexerMode.Intake)) {
            colorSensor.update();
        }



//        if () {}

        bootkickerFSM();
        handleIndexingMode();
        sorterFSM();

    }


    //**
    // This update class is for teleop
    // *//

    public void update(Gamepad gamepad1) { //this is for teleop
        if (!isInitalized) { //ALWAYS BRING BOOTKICKER DOWN AFTER A RUN ALWAYS!!!!! NEVER LET IT ALIGN ON A WALL
            bootkicker.setPosition(0);
            canSpin = true;

            spindexerMode = SpindexerMode.Intake;
            sorter.setTargetRotation(intakePositions[0]);

            //FSM states initalization
            kickerState = KickerState.Ready;
            sorterState = SorterState.Ready;

            currentChamber = 0;
            isInitalized = true;
            return;
        }

        sorter.update();

        if (gamepad1.aWasReleased()) { // choose motif from button presses, have this add to telem, prob deprecate this in final
            switch (teleopMotif) {
                case 1:
                    motif = new String[]{"G", "P", "P"};
                    teleopMotif++;
                    break;
                case 2:
                    motif = new String[]{"P", "G", "P"};
                    teleopMotif++;
                    break;
                case 3:
                    motif = new String[]{"P", "G", "G"};
                    teleopMotif = 4;
                    break;
            }
        }

        if (gamepad1.xWasReleased()) {
            spindexerMode = SpindexerMode.Outtake;
        }

        if (gamepad1.backWasReleased()) {
            spindexerMode = SpindexerMode.Intake;
        }

        //check in Intake mode, the spindexer is not changing target or moving, kicker isnt doing anything,
        if (gamepad1.dpadUpWasReleased() && kickerState.equals(KickerState.Ready)
                && sorterState.equals(SorterState.Ready) && spindexerMode.equals(SpindexerMode.Intake)) {
            //call fsm for servo
            kickerState = KickerState.SendUp;
        }

        bootkickerFSM();
        handleIndexingMode();
        sorterFSM();
        //shooting modes
    }

    public void bootkickerFSM() {
        switch (kickerState) {
            case Ready:
                bootkickerCalled = false;
                break;
            case SendUp:
                bootkickerCalled = true;
                bootkicker.setPosition(0.45);
                kickerState = KickerState.SendDown;
                bootKickerTimer.reset();
                break;
            case SendDown:
                if (bootKickerTimer.milliseconds() >= BOOTKICKER_DELAY) {
                    bootkicker.setPosition(0);
                    kickerState = KickerState.WaitTillDown;
                    bootKickerTimer.reset();
                }
                break;
            case WaitTillDown:
                if (bootKickerTimer.milliseconds() >= BOOTKICKER_DELAY) {
                    kickerState = KickerState.Ready;
                }
                break;
        }
    }




    //Angle Rotations for intake:
    //Slot 1: 60?
    //slot 2; 120?
    //slot 3: 340?

    //Angle Rotations for Outtake/under flywheel
    //slot 1: 200 or 220??
    //slot 2:
    //slot 3: 30
    //https://docs.ftclib.org/ftclib/features/util#timing-functions replace timers with this

    public void sorterFSM() {
        switch (sorterState) {
            case Ready:
                break;
            case Spinning:
                if (sorter.isAtTarget()) {
                    sorterState = SorterState.Ready;
                }
                break;
            case SpinToEmptyChamber: //only use for intake
                for (int i = 0; i < 3; i++) {
                    if (inventory[i].equals("E")) {
                        currentChamber = i;
                        sorter.setTargetRotation(intakePositions[i]);
                        sorterState = SorterState.Spinning;
                        break;
                    }
                }
                break;
            case SpinToOuttakeTargetingMotif:
                // doesnt account if nothing in inventory matches
                int targetPos = getTargetChamberForMotif();
                sorter.setTargetRotation(outTakePositions[targetPos]);
                inventory[targetPos] = "E";
                sorterState = SorterState.Spinning;
                break;
        }
    }

    public int getTargetChamberForMotif () {
        for (int i = 0; i < 3; i ++) {
            if (inventory[i].equals(motif[currentTargetMotifNum])) {
                currentTargetMotifNum = currentTargetMotifNum + 1 % 3;
                return i;
            }
        }
        for (int j = 0; j < 3; j ++) {
            if (!inventory[j].equals("E")) {
                currentTargetMotifNum = currentTargetMotifNum + 1 % 3;
                return j;
            }
        }
        return 0;

    }

    public boolean canSpin() {
        return (!bootkickerCalled && sorterState.equals(SorterState.Ready));
    }

    public void handleIndexingMode() {
        if (!canSpin()) return;
        if (spindexerMode.equals(SpindexerMode.Outtake)) {
            if (readyToShoot()) {
                shootSequenceState = ShootSequenceState.ChangeChamber;
            }
            AutoShootingSequenceAuton();
        }
        else if (spindexerMode.equals(SpindexerMode.Intake) && !isFull()) {
            colorSensor.update(); //call colorSensor.update() to
        }
    }

    public void AutoShootingSequenceAuton () {
        //the delay checks should allow for enough time for the flywheel to return to targetVelocity

        if (isEmpty()) { //
            shootSequenceState = ShootSequenceState.Ready; //last step where it switches back to IntakeMode and goes to empty Chamber 1
            sorterState = SorterState.SpinToEmptyChamber;
            spindexerMode = SpindexerMode.Intake;
            return;
        }

        switch (shootSequenceState) { //in order to start set shootSequenceState to ChangeChamber
            case Ready :
                break;
            case ChangeChamber: //if shootStep = 1 initate
                if (sorterState.equals(SorterState.Ready)) {
                    sorterState = sorterState.SpinToOuttakeTargetingMotif;
                    shootSequenceState = ShootSequenceState.KickArtifact;
                    shootSequenceTimer.reset();
                    break;
                }
                break;
            case KickArtifact:
                if (sorterState.equals(SorterState.Ready) && shootSequenceTimer.milliseconds() >= 300) { //create a timer and then check if greater than certain time
                    kickerState = KickerState.SendUp;
                    shootSequenceState = ShootSequenceState.WaitForKicker;
                    shootSequenceTimer.reset();
                }
                break;
            case WaitForKicker:
                if (shootSequenceTimer.milliseconds() >= (BOOTKICKER_DELAY * 2) + 25) {
                    shootSequenceState = ShootSequenceState.ChangeChamber;
                }
                break;



        }

    }

    private boolean readyToShoot() {
        return true;
    }

    public class ColorSensor {
        private final RevColorSensorV3 revColorSensor;
        public ColorSensor(RevColorSensorV3 revColorSensor) {
            this.revColorSensor = revColorSensor;
        }

        public void update() {
//         NormalizedRGBA colors = colorSensor.getNormalizedColors();
//         RGB = new double[] {colors.red, colors.green, colors.blue};
            String colorDetected = detectArtifactColor();

            if (colorDetected.equals("E")) {
                colorFound = false;
                return;
            }

            if (!colorFound) {
                colorFound = true;
                colorSensorTimer.reset();
                colorStartTime = colorSensorTimer.milliseconds();
            }

            if (colorSensorTimer.milliseconds() - colorStartTime >= TIME_TO_DETECT) {
                if (inventory[currentChamber].equals("E")) {
                    inventory[currentChamber] = colorDetected;
                    sorterState = SorterState.SpinToEmptyChamber;
                    flagActive = "flaged";
                }
                colorStartTime = 0;
                colorFound = true;
                //deactivate colorSensor
            }
            else {
                flagActive = "Not flaged";
            }
        }
        //TODO: RETUNE COLOR SENSOR
        //TODO: Optional get feedback from the GoBuilda light when updating/detecting a artifact
        public String detectArtifactColor() {
            r = revColorSensor.red(); //test if you dont need remove g> r and b > r and see if it still works
            g = revColorSensor.green();
            b = revColorSensor.blue();
            opacity = getARGB();
            if (opacity >= 300000000) { // higher opacity means no object detected
                return "E";
            } else if (g > r && g > b && g > 50 && g < 600) { //sometimes g or b will go to some absurd value in the hundreds
                return "G";
            } else if (b > r && b > g && b > 50 && b < 600) { // for purple check
                return "P";
            } else return "E";
        }

        public int getARGB() {
            return revColorSensor.argb();
        }

        public String log() {

            return "Red: " + r
                    + "\nGreen: " + g
                    + "\nBlue: " + b
                    + "\nARGB: " + getARGB()
                    + "\nArtifact Color: " + detectArtifactColor() +
                    "Is able to spin: " + canSpin;
        }
    }

    public void setMotif(int motifTagNum) { //may not be needed
        if (motifTagNum == 21) {
            motif = new String[]{"G", "P", "P"};
        } else if (motifTagNum == 22) {
            motif = new String[]{"P", "G", "P"};
        } else if (motifTagNum == 23) {
            motif = new String[]{"P", "P", "G"};
        }
    }



    public String printInventory () {
        return "Chamber 1: " + inventory[0] + "\n Chamber 2: " +  inventory[1] + "\n Chamber 3: " + inventory[2];
    }

    public boolean isFull() {
        for (int i = 0; i < 3; i++) {
            if (inventory[i].equals("E")) {
                return false;
            }
        }
        return true;
    }

    private boolean isEmpty() {
        for (int i = 0; i < 3; i++) {
            if (inventory[i].equals("P") || inventory[i].equals("G")) {
                return false;
            }
        }
        return true;
    }
    public boolean checkInventory(int targetColor) {
        for (int j = 0; j < 3; j++) {
            if (inventory[j].equals(motif[targetColor])) {
                sorter.changeTargetRotation(outTakePositions[j]);
                return true;
            }
        }
        return false;
    }

//    public void feedFromIntake () {

    private enum SpindexerMode {
        Intake,
        Outtake
    }


    private enum SorterState {
        Ready,
        SpinToEmptyChamber,
        Spinning,
        SpinToOuttakeTargetingMotif
    }

    private enum KickerState {
        Ready,
        SendUp,
        SendDown,
        WaitTillDown
    }

    private enum ShootSequenceState {
        Ready,
        ChangeChamber,
        WaitForKicker,
        KickArtifact
    }

    @TeleOp(name = "Test Spindexer", group = "test")
    public static class ColorSensorTest extends LinearOpMode {

        @Override
        public void runOpMode() {

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

                telemetry.addLine(newSpindexer.colorSensor.log());
                telemetry.addData("x: ", newSpindexer.x);
                telemetry.addData("Pos State: ", newSpindexer.posState);
                telemetry.addData("Spindexer Power", newSpindexer.sorter.getPower());
                telemetry.addLine(newSpindexer.flagActive);
                telemetry.addLine(newSpindexer.printInventory());
                telemetry.update();
            }
        }
    }
}
