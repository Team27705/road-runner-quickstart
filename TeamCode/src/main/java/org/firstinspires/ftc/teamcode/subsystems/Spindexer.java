package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.hardware.RTPTorctex;

import java.util.Arrays;

public class Spindexer {
    //include spindexer servo, light, booktkicker, and color sensor
    //note:deprecate Spindexer.java once done

    public String[] motif = new String[]{"G", "P", "P"};
    public String x;
    public String posState;

    // Hardware
    private final RTPTorctex sorter;
    private final Bootkicker bootkicker;
    private final ColorSensor colorSensor;

    // Constants
    private final int[] outTakePositions = {190, 310, 70}; //index 0 is the degree to send slot 1 to intake, etc
    private final int[] intakePositions = {10, 130, 250}; //index 0 is the degree to send slot 1 to outtake, etc
    // Bot Variables
    public String[] inventory = {"E", "E", "E"}; //E = empty, P = purple, G = green
    private boolean isInitialized;

    // State Variable
    private boolean canSpin;
    private SpindexerMode spindexerMode;
    private SorterState sorterState;
    private ShootSequenceState shootSequenceState;
    public int currentChamber;
    private int teleopMotif;
    private int currentTargetMotifNum = 0;


    //https://gm0.org/en/latest/docs/software/concepts/finite-state-machines.html
    //replace the ifs with a switch case
    public Spindexer(HardwareMap hardwareMap, boolean auto) {
        //Spindexer Servo
        CRServo crServo = hardwareMap.get(CRServo.class, "Spindexer Servo");
        AnalogInput analogEncoder = hardwareMap.get(AnalogInput.class, "Spindexer Encoder");
        sorter = new RTPTorctex(crServo, analogEncoder);
        RevColorSensorV3 revColorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor = new ColorSensor(revColorSensor, this);

        bootkicker = new Bootkicker(hardwareMap);

        shootSequenceState = ShootSequenceState.Ready;
        if (auto) {
            inventory = new String[] {"P", "G", "P"}; //Todo: physically label chambers 1, 2, 3
            spindexerMode = SpindexerMode.Outtake;
        }
        else {
            spindexerMode = SpindexerMode.Intake;
        }

        isInitialized = false;
    }

    //https://github.com/NgDodo/FTC12209new/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Sorter.java#L17

    //**
    // This update class is for auton
    // *//

    public void update() {
        if (!isInitialized) {
            bootkicker.initialize();
            canSpin = true;

            spindexerMode = SpindexerMode.AutonWait;

            sorterState = SorterState.Ready;

            isInitialized = true;
//            currentChamber = 0;
            return;
        }

        if (colorSensor.artifactDetected) {
            sorterState = SorterState.SpinToEmptyChamber;
        }

        //only let the Torctex PID update if the kicker is down. May have to forcibly set power in a else {.setPower(0);}
        if (bootkicker.isReady()) {
            sorter.update();
        }
        else {
            sorter.setPower(0);
        }

        if (spindexerMode == SpindexerMode.AutonWait) return;

        if (spindexerMode == SpindexerMode.Outtake) {
            AutoShootingSequenceAuton(); // may have to create a flag in !
        }
        else if (spindexerMode == SpindexerMode.Intake) { //maybe check if flywheel is currently within target deadzone?
            colorSensor.update();
        }

        bootkicker.update();
        handleIndexingMode();
        sorterFSM();

    }


    //use this for actions in auton, call update() at the start of the loop and then call this when
    //you want to
    public void changeSpindexerMode(SpindexerMode newMode) {
        this.spindexerMode = newMode;
    }

    /**
     * Overloaded `update()` for teleop use. Note the addition of gamepad as input
     */
    public void update(Gamepad gamepad2) {
        if (!isInitialized) { //ALWAYS BRING BOOTKICKER DOWN AFTER A RUN ALWAYS!!!!! NEVER LET IT ALIGN ON A WALL
            bootkicker.initialize();
            canSpin = true;

            spindexerMode = SpindexerMode.Intake;
            sorter.setTargetRotation(intakePositions[0]);

            //FSM states initalization
            sorterState = SorterState.Ready;

            currentChamber = 0;
            isInitialized = true;
            return;
        }

        if (bootkicker.isReady()) {
            sorter.update();
        }
        else {
            sorter.setPower(0);
        }

        if (colorSensor.artifactDetected) {
            sorterState = SorterState.SpinToEmptyChamber;
        }

        if (gamepad2.aWasReleased()) { // choose motif from button presses, have this add to telem, prob deprecate this in final
            switch (teleopMotif) {
                case 0:
                    motif = new String[]{"G", "P", "P"};
                    teleopMotif++;
                    break;
                case 1:
                    motif = new String[]{"P", "G", "P"};
                    teleopMotif++;
                    break;
                case 2:
                    motif = new String[]{"P", "P", "G"};
                    teleopMotif = 0;
                    break;
            }
        }

        if (gamepad2.xWasReleased()) {
            setSpindexerMode(SpindexerMode.Outtake);
        }

        if (gamepad2.backWasReleased()) {
            setSpindexerMode(SpindexerMode.Intake);
        }

        //check in Intake mode, the spindexer is not changing target or moving, kicker isnt doing anything,
        if (gamepad2.dpadUpWasReleased() && bootkicker.isReady()
                && sorterState == SorterState.Ready && spindexerMode == SpindexerMode.Intake) {
            //call fsm for servo
            bootkicker.kick();
        }

        switch (spindexerMode) {
            case Intake:
                gamepad2.setLedColor(0, 1, 0, 100000);
                break;
            case Outtake:
                gamepad2.setLedColor(1, 0, 0, 100000);
                break;
        }

        bootkicker.update();
        handleIndexingMode();
        sorterFSM();
        //shooting modes
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

    public int getTargetChamberForMotif() {
        for (int i = 0; i < 3; i++) {
            if (inventory[i].equals(motif[currentTargetMotifNum])) {
                // REMOVED: inventory[targetPos] = "E";
                return i;
            }
        }
        for (int j = 0; j < 3; j++) {
            if (!inventory[j].equals("E")) {
                return j;
            }
        }
        return 0;
    }

    public boolean canSpin() {
        return (!bootkicker.isCalled() && sorterState == SorterState.Ready);
    }

    public void setSpindexerMode(SpindexerMode mode) {
        this.spindexerMode = mode;
        if (mode == SpindexerMode.Outtake) {
            // Force the state machine to start
            this.currentTargetMotifNum = 0;
            this.shootSequenceState = ShootSequenceState.ChangeChamber; // Start immediately
            this.sorterState = SorterState.Ready; // Interrupt any current spin
        } else {
            this.sorterState = SorterState.SpinToEmptyChamber;
        }
    }

    public void handleIndexingMode() {
        if (!canSpin()) return;

        if (spindexerMode == SpindexerMode.Outtake) {
            // Just run the sequence; the mode-switch handles the start state
            AutoShootingSequenceAuton();
        }
        else if (spindexerMode == SpindexerMode.Intake && !isFull()) {
            colorSensor.update();
        }
    }

    public void AutoShootingSequenceAuton() {
        // Only exit if we are Ready (not in the middle of a shot) AND empty
        if (shootSequenceState == ShootSequenceState.Ready && isEmpty()) {
            spindexerMode = SpindexerMode.Intake;
            currentTargetMotifNum = 0;
            sorterState = SorterState.SpinToEmptyChamber;
            return;
        }

        switch (shootSequenceState) {
            case Ready:
                if (spindexerMode == SpindexerMode.Outtake && !isEmpty()) {
                    shootSequenceState = ShootSequenceState.ChangeChamber;
                }
                break;

            case ChangeChamber:
                if (sorterState == SorterState.Ready && bootkicker.isReady()) {
                    // Determine target
                    int targetPos = getTargetChamberForMotif();
                    sorter.setTargetRotation(outTakePositions[targetPos]);

                    // Store which chamber we are about to shoot so we can empty it later
                    currentChamber = targetPos;

                    sorterState = SorterState.Spinning; // Set to spinning so we wait for arrival
                    shootSequenceState = ShootSequenceState.KickArtifact;
                }
                break;

            case KickArtifact:
                if (sorterState == SorterState.Ready) {
                    bootkicker.kick();
                    // NOW we mark it as empty, after the sorter arrived and kicker started
                    inventory[currentChamber] = "E";
                    currentTargetMotifNum = (currentTargetMotifNum + 1) % 3;

                    shootSequenceState = ShootSequenceState.WaitForKicker;
                }
                break;

            case WaitForKicker:
                if (bootkicker.isReady()) {
                    shootSequenceState = ShootSequenceState.Ready;
                }
                break;
        }
    }

    private boolean readyToShoot() {
        return bootkicker.isReady() && sorterState == SorterState.Ready;
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
        Outtake,
        AutonWait
    }


    private enum SorterState {
        Ready,
        SpinToEmptyChamber,
        Spinning,
        SpinToOuttakeTargetingMotif
    }


    private enum ShootSequenceState {
        Ready,
        ChangeChamber,
        WaitForKicker,
        KickArtifact
    }

    @TeleOp(name = "Test Spindexer", group = "Testing")
    public static class ColorSensorTest extends LinearOpMode {

        @Override
        public void runOpMode() {

            waitForStart();
            Spindexer spindexer = new Spindexer(this.hardwareMap, false);


            while (!isStopRequested()) {
                spindexer.update(gamepad1);

//                if (gamepad1.dpadDownWasReleased()) {
//                    spindexer.resetServo();
//                }
//                if (gamepad1.dpadUpWasReleased()){
//                    spindexer.kick();
//                }

                telemetry.addLine(spindexer.colorSensor.log());
                telemetry.addData("x: ", spindexer.x);
                telemetry.addData("Pos State: ", spindexer.posState);
                telemetry.addData("Spindexer Power", spindexer.sorter.getPower());
                telemetry.addData("Artifact Detected", spindexer.colorSensor.isArtifactDetected());
                telemetry.addLine(spindexer.printInventory());
                telemetry.addData("Inventory", Arrays.toString(spindexer.inventory));
                telemetry.addData("Shoot State", spindexer.shootSequenceState);
                telemetry.update();
            }
        }
    }
}
