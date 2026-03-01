package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Sorter.ChamberState;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterMode;
import org.firstinspires.ftc.teamcode.subsystems.hardwares.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.hardwares.ColorSensor.DetectedColor;

/**
 * NewSpindexer coordinates the {@link Sorter}, {@link Bootkicker}, and
 * {@link ColorSensor} subsystems to sort incoming artifacts (intake mode) and
 * fire them either in motif order or in sequential chamber order (outtake mode).
 *
 * <h3>Critical safety rule</h3>
 * The {@link Sorter} servo is locked at zero power whenever the
 * {@link Bootkicker} is in motion. This prevents the kicker arm from jamming
 * against the rotating sorter drum and causing physical damage.
 *
 * <h3>Usage</h3>
 * Call {@link #update()} (autonomous) or {@link #update(Gamepad)} (TeleOp)
 * exactly once per loop iteration. Do not call both in the same loop.
 */
public class NewSpindexer {

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------

    /**
     * The color sensor must read the same color for this many milliseconds
     * before the detection is committed to the inventory. Prevents transient
     * reflections from being logged as real artifacts.
     */
    private static final int COLOR_DEBOUNCE_MS = 25;

    // -------------------------------------------------------------------------
    // Hardware subsystems
    // -------------------------------------------------------------------------

    private final Sorter sorter;
    private final Bootkicker bootkicker;
    private final ColorSensor colorSensor;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    /**
     * The expected color sequence for the three chambers. Defaults to
     * {@code {"G","P","P"}}. Can be overridden via {@link #setMotif(String[])}.
     * <p>
     * "G" = green, "P" = purple.
     */
    public static String[] motif = new String[]{"G", "P", "P"};

    private volatile SpindexerMode spindexerMode;
    private volatile ShootSequenceState shootSequenceState = ShootSequenceState.Ready;

    /** Index of the chamber currently positioned under the intake or outtake port. */
    private int currentChamber = 0;

    /** Position within {@link #motif} for the next artifact to be fired. */
    private int currentMotifIndex = 0;

    /** Whether the current outtake pass respects the motif order or fires sequentially. */
    private OuttakeMode outtakeMode = OuttakeMode.Motif;

    /**
     * True when the color sensor has seen an artifact but the debounce window
     * has not yet elapsed.
     */
    private boolean colorFound = false;
    private final ElapsedTime colorSensorTimer = new ElapsedTime();

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * Creates a NewSpindexer and initialises all three hardware subsystems.
     *
     * @param hardwareMap the robot hardware map
     * @param auto        {@code true} when called from an autonomous OpMode;
     *                    pre-loads the inventory and starts in outtake mode
     */
    public NewSpindexer(HardwareMap hardwareMap, boolean auto) {
        sorter = new Sorter(hardwareMap);
        bootkicker = new Bootkicker(hardwareMap);
        colorSensor = new ColorSensor(hardwareMap);

        if (auto) {
            // Chambers are pre-loaded before the match starts.
            sorter.setInventory(new ChamberState[]{
                    ChamberState.Purple, ChamberState.Green, ChamberState.Purple
            });
            sorter.setSorterMode(SorterMode.Outtake);
            spindexerMode = SpindexerMode.Outtake;
            shootSequenceState = ShootSequenceState.ChangeChamber;
        } else {
            sorter.setSorterMode(SorterMode.Intake);
            spindexerMode = SpindexerMode.Intake;
        }
    }

    // -------------------------------------------------------------------------
    // Public API — update methods
    // -------------------------------------------------------------------------

    /**
     * Drives all state machines and hardware subsystems. Must be called once
     * per loop iteration in autonomous OpModes.
     * <p>
     * The sorter servo is automatically held at zero power while the bootkicker
     * is in motion.
     */
    public synchronized void update() {
        tickSubsystems();

        switch (spindexerMode) {
            case Intake:
                tickIntake();
                break;
            case Outtake:
                tickShootSequence();
                break;
            case Idle:
                break;
        }
    }

    /**
     * Drives all state machines with additional gamepad input for TeleOp.
     * Must be called once per loop iteration; do not call {@link #update()}
     * separately in the same loop.
     *
     * <ul>
     *   <li>dpad-up: manual single kick (only active in Outtake mode)</li>
     *   <li>x: switch to Outtake/shoot mode</li>
     *   <li>back: switch to Intake mode</li>
     * </ul>
     *
     * @param gamepad the driver gamepad (typically gamepad2)
     */
    public synchronized void update(Gamepad gamepad) {
        if (gamepad.dpadUpWasReleased() && spindexerMode == SpindexerMode.Outtake) {
            kick();
        }
        if (gamepad.xWasReleased()) {
            startOuttake();
        }
        if (gamepad.backWasReleased()) {
            startIntake();
        }

        // LED colour gives instant mode feedback
        switch (spindexerMode) {
            case Intake:
                gamepad.setLedColor(0, 1, 0, 100_000);
                break;
            case Outtake:
                gamepad.setLedColor(1, 0, 0, 100_000);
                break;
            default:
                gamepad.setLedColor(0, 0, 1, 100_000);
                break;
        }

        update();
    }

    // -------------------------------------------------------------------------
    // Public API — mode control
    // -------------------------------------------------------------------------

    /**
     * Switches to intake mode and positions the sorter at the first empty
     * chamber. Safe to call from any mode at any time.
     */
    public synchronized void startIntake() {
        spindexerMode = SpindexerMode.Intake;
        colorFound = false;
        sorter.setSorterMode(SorterMode.Intake);
        goToNextEmptyChamber();
    }

    /**
     * Switches to outtake/shoot mode in {@link OuttakeMode#Motif} order and
     * immediately begins the automatic shoot sequence. Safe to call from any mode.
     */
    public synchronized void startOuttake() {
        startOuttake(OuttakeMode.Motif);
    }

    /**
     * Switches to outtake/shoot mode with the specified firing strategy and
     * immediately begins the automatic shoot sequence. Safe to call from any mode.
     *
     * @param mode {@link OuttakeMode#Motif} to fire in the motif-defined color order,
     *             or {@link OuttakeMode#Any} to fire all chambers sequentially
     *             regardless of color
     */
    public synchronized void startOuttake(OuttakeMode mode) {
        spindexerMode = SpindexerMode.Outtake;
        outtakeMode = mode;
        currentMotifIndex = 0;
        sorter.setSorterMode(SorterMode.Outtake);
        shootSequenceState = ShootSequenceState.ChangeChamber;
    }

    /**
     * Stops all active operation. The sorter servo holds its last position;
     * the bootkicker will complete any kick cycle already in progress.
     */
    public synchronized void setIdle() {
        spindexerMode = SpindexerMode.Idle;
    }

    /**
     * Triggers a single kick. No-ops if the bootkicker is already in motion
     * or the sorter has not yet settled at its target position. This prevents
     * the kicker from striking an artifact that is still moving into position.
     */
    public synchronized void kick() {
        if (bootkicker.isReady() && sorter.isAtTarget()) {
            bootkicker.kick();
        }
    }

    // -------------------------------------------------------------------------
    // Public API — inventory and configuration
    // -------------------------------------------------------------------------

    /**
     * Returns {@code true} when all three chambers contain an artifact.
     */
    public boolean isFull() {
        for (ChamberState state : sorter.getInventory()) {
            if (state == ChamberState.Empty) return false;
        }
        return true;
    }

    /**
     * Returns {@code true} when all three chambers are empty.
     */
    public boolean isEmpty() {
        for (ChamberState state : sorter.getInventory()) {
            if (state != ChamberState.Empty) return false;
        }
        return true;
    }

    /**
     * Returns the live inventory array (index 0 = chamber 1, etc.).
     * Modifications to the returned array will affect internal state.
     */
    public ChamberState[] getInventory() {
        return sorter.getInventory();
    }

    /**
     * Replaces the current inventory. Used during autonomous to declare which
     * chambers are pre-loaded.
     *
     * @param inventory three {@link ChamberState} values
     */
    public synchronized void setInventory(ChamberState[] inventory) {
        sorter.setInventory(inventory);
    }

    /**
     * Changes the shoot motif at runtime. Each element must be {@code "G"}
     * (green) or {@code "P"} (purple).
     *
     * @param newMotif three-element array, e.g. {@code {"G","P","P"}}
     */
    public synchronized void setMotif(String[] newMotif) {
        motif = newMotif;
    }

    /**
     * Returns a multi-line string suitable for FTC telemetry.
     */
    public String log() {
        ChamberState[] inv = sorter.getInventory();
        return "Mode          : " + spindexerMode
                + "\nOuttake Mode  : " + outtakeMode
                + "\nShoot State   : " + shootSequenceState
                + "\nCurrent Chamber: " + (currentChamber + 1)
                + "\nInventory     : [" + inv[0] + ", " + inv[1] + ", " + inv[2] + "]"
                + "\nBootkicker rdy: " + bootkicker.isReady()
                + "\nSorter at tgt : " + sorter.isAtTarget()
                + "\nColor detected: " + colorSensor.detectArtifactColor();
    }

    // -------------------------------------------------------------------------
    // Internal — subsystem tick
    // -------------------------------------------------------------------------

    /**
     * Updates the hardware subsystems each loop.
     *
     * <p><b>Safety:</b> the sorter servo is forced to zero power while the
     * bootkicker is active. The bootkicker arm physically occupies the sorter
     * drum's path; allowing the drum to spin would cause a mechanical jam.
     */
    private void tickSubsystems() {
        if (bootkicker.isReady()) {
            // Bootkicker is idle — allow the sorter PID to run normally.
            sorter.update();
        } else {
            // Bootkicker is in motion — lock the sorter to prevent jams.
            sorter.setPower(0);
        }
        bootkicker.update();
    }

    // -------------------------------------------------------------------------
    // Internal — intake FSM
    // -------------------------------------------------------------------------

    /**
     * Runs the intake logic: reads the color sensor with debouncing and fills
     * the inventory chamber by chamber. Automatically switches to outtake mode
     * once all chambers are full.
     */
    private void tickIntake() {
        if (isFull()) {
            startOuttake();
            return;
        }

        DetectedColor detected = colorSensor.detectArtifactColor();

        if (detected == DetectedColor.EMPTY) {
            colorFound = false;
            return;
        }

        // Start the debounce window on the first frame we see an artifact.
        if (!colorFound) {
            colorFound = true;
            colorSensorTimer.reset();
            return;
        }

        if (colorSensorTimer.milliseconds() < COLOR_DEBOUNCE_MS) {
            return; // Debounce not yet elapsed.
        }

        // Confirmed artifact — commit to inventory and move to next empty slot.
        ChamberState state = (detected == DetectedColor.GREEN)
                ? ChamberState.Green
                : ChamberState.Purple;
        sorter.setInventory(currentChamber, state);
        colorFound = false;

        if (!isFull()) {
            goToNextEmptyChamber();
        }
    }

    // -------------------------------------------------------------------------
    // Internal — shoot sequence FSM
    // -------------------------------------------------------------------------

    /**
     * Runs the shoot sequence FSM during outtake mode.
     *
     * <pre>
     * Ready ──► ChangeChamber ──► WaitForSorter ──► KickArtifact ──► WaitForKicker
     *                ▲                                                      │
     *                └──────────────────────────────────────────────────────┘
     *                          (loops until inventory is empty)
     * </pre>
     *
     * Transitions back to intake automatically once all chambers have been fired.
     */
    private void tickShootSequence() {
        if (isEmpty()) {
            startIntake();
            return;
        }

        switch (shootSequenceState) {
            case Ready:
                // A mode switch or external call can kick us off from here.
                if (!isEmpty()) {
                    shootSequenceState = ShootSequenceState.ChangeChamber;
                }
                break;

            case ChangeChamber:
                // Wait until both the bootkicker is idle and the sorter has settled
                // before commanding a new target, to avoid mid-move conflicts.
                if (bootkicker.isReady() && sorter.isAtTarget()) {
                    currentChamber = getTargetChamber();
                    sorter.goToChamber(currentChamber);
                    shootSequenceState = ShootSequenceState.WaitForSorter;
                }
                break;

            case WaitForSorter:
                if (sorter.isAtTarget()) {
                    shootSequenceState = ShootSequenceState.KickArtifact;
                }
                break;

            case KickArtifact:
                // Sorter is confirmed at target. Fire and mark the chamber empty.
                bootkicker.kick();
                sorter.setInventory(currentChamber, ChamberState.Empty);
                if (outtakeMode == OuttakeMode.Motif) {
                    currentMotifIndex = (currentMotifIndex + 1) % motif.length;
                }
                shootSequenceState = ShootSequenceState.WaitForKicker;
                break;

            case WaitForKicker:
                if (bootkicker.isReady()) {
                    shootSequenceState = isEmpty()
                            ? ShootSequenceState.Ready
                            : ShootSequenceState.ChangeChamber;
                }
                break;
        }
    }

    // -------------------------------------------------------------------------
    // Internal — helpers
    // -------------------------------------------------------------------------

    /**
     * Returns the next chamber index to fire based on the current {@link OuttakeMode}.
     * <ul>
     *   <li>{@link OuttakeMode#Motif}: picks the chamber whose color matches
     *       {@code motif[currentMotifIndex]}, falling back to the first non-empty
     *       chamber if no match exists.</li>
     *   <li>{@link OuttakeMode#Any}: picks the first non-empty chamber in index
     *       order (0 → 1 → 2), ignoring color.</li>
     * </ul>
     */
    private int getTargetChamber() {
        ChamberState[] inv = sorter.getInventory();

        if (outtakeMode == OuttakeMode.Any) {
            for (int i = 0; i < inv.length; i++) {
                if (inv[i] != ChamberState.Empty) return i;
            }
            return 0; // Guarded by isEmpty() check in tickShootSequence().
        }

        // Motif mode: try to match the desired color first.
        String target = motif[currentMotifIndex];
        ChamberState desired = target.equals("G") ? ChamberState.Green : ChamberState.Purple;

        for (int i = 0; i < inv.length; i++) {
            if (inv[i] == desired) return i;
        }

        // Fallback: any non-empty chamber to avoid stalling.
        for (int i = 0; i < inv.length; i++) {
            if (inv[i] != ChamberState.Empty) return i;
        }

        return 0; // Guarded by isEmpty() check in tickShootSequence().
    }

    /**
     * Advances the sorter to the first chamber that does not contain an artifact.
     * Called at the start of intake mode and after each artifact is logged.
     */
    private void goToNextEmptyChamber() {
        ChamberState[] inv = sorter.getInventory();
        for (int i = 0; i < inv.length; i++) {
            if (inv[i] == ChamberState.Empty) {
                currentChamber = i;
                sorter.goToChamber(i);
                return;
            }
        }
    }

    // -------------------------------------------------------------------------
    // Enums
    // -------------------------------------------------------------------------

    private enum SpindexerMode {
        /** Color sensor is active; incoming artifacts are logged to inventory. */
        Intake,
        /** Shoot sequence FSM is active; artifacts are fired in motif order. */
        Outtake,
        /** Subsystems are held in place but no state machine is running. */
        Idle
    }

    /**
     * Controls how the spindexer selects which chamber to fire next during outtake.
     */
    public enum OuttakeMode {
        /** Fire chambers in the color order defined by {@link #motif}. */
        Motif,
        /** Fire all chambers sequentially (index 0 → 1 → 2) regardless of color. */
        Any
    }

    private enum ShootSequenceState {
        Ready,
        ChangeChamber,
        WaitForSorter,
        KickArtifact,
        WaitForKicker
    }
}
