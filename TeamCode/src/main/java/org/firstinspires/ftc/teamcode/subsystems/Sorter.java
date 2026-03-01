package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.hardwares.RTPTorctex;

import java.util.NoSuchElementException;

/**
 * The Sorter subsystem manages the servo and inventory for the three-chamber sorter mechanism. It
 * allows switching between intake and outtake modes, tracking the state of each chamber, and moving
 * the servo to the appropriate position based on the current mode and desired chamber.
 */
public class Sorter {

    //----------------------------------------------------------------------------------------------
    // Constants
    //----------------------------------------------------------------------------------------------

    private static final int NUM_CHAMBERS = 3;

    // These positions are empirically determined and may need to be adjusted.
    // The index corresponds to the chamber number: index 0 is for slot 1, etc.
    // The number in index 0 should be the position that sends the first chamber
    // to the outtake or intake, etc.
    private static final int[] outtakePositions = {190, 310, 70};
    private static final int[] intakePositions = {10, 130, 250};

    private final RTPTorctex sorter;

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------
    private int currentChamber = 0;
    private SorterMode currentMode = SorterMode.Intake;
    private ChamberState[] inventory = {ChamberState.Empty, ChamberState.Empty, ChamberState.Empty};

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    /**
     * Initializes the sorter subsystem. The servo should be configured in the
     * robot configuration with the name "Spindexer Servo", and the analog encoder
     * should be configured with the name "Spindexer Encoder".
     *
     * @param hardwareMap the hardware map to pull the servo and encoder from
     */
    public Sorter(HardwareMap hardwareMap) {
        CRServo crServo = hardwareMap.get(CRServo.class, "Spindexer Servo");
        AnalogInput analogEncoder = hardwareMap.get(AnalogInput.class, "Spindexer Encoder");
        sorter = new RTPTorctex(crServo, analogEncoder);
    }

    //----------------------------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------------------------

    /**
     * Sets the sorter to the specified mode, which moves the servo to the corresponding position.
     *
     * @param mode the {@link SorterMode} to set
     * @throws IllegalArgumentException if the mode is invalid
     */
    public void setSorterMode(SorterMode mode) {
        currentMode = mode;
        int targetRotation;
        switch (mode) {
            case Intake:
                targetRotation = intakePositions[currentChamber];
                break;
            case Outtake:
                targetRotation = outtakePositions[currentChamber];
                break;
            default:
                throw new IllegalArgumentException("Invalid SorterMode: " + mode);
        }
        sorter.setTargetRotation(targetRotation);
    }

    /**
     * Returns the internal inventory array representing the three chambers.
     * Note: this returns a direct reference to the internal array.
     *
     * @return the inventory array
     * @see #getInventory(int) for accessing individual chamber states
     * @see #setInventory(ChamberState[]) for replacing the entire inventory
     * @see #setInventory(int, ChamberState) for setting individual chamber states
     */
    public ChamberState[] getInventory() {
        return inventory;
    }

    /**
     * Replaces the internal inventory array with the provided array.
     *
     * @param inventory the new inventory array
     * @see #setInventory(int, ChamberState) for setting individual chamber states
     * @see #getInventory() for accessing the entire inventory array
     * @see #getInventory(int) for accessing individual chamber states
     */
    public void setInventory(ChamberState[] inventory) {
        this.inventory = inventory;
    }

    /**
     * Returns the state of the chamber at the given index.
     *
     * @param index the chamber index (0..NUM_CHAMBERS-1)
     * @return the {@link ChamberState} at the specified index
     * @throws IllegalArgumentException if the index is out of bounds
     * @see #getInventory() for accessing the entire inventory array
     * @see #setInventory(ChamberState[]) for replacing the entire inventory
     * @see #setInventory(int, ChamberState) for setting individual chamber states
     */
    public ChamberState getInventory(int index) {
        if (index < 0 || index >= NUM_CHAMBERS) {
            throw new IllegalArgumentException("Index out of bounds");
        }
        return inventory[index];
    }

    /**
     * Sets the state of a single chamber.
     *
     * @param index the chamber index (0..inventory.length-1)
     * @param color the {@link ChamberState} to set; must not be {@code null}
     * @throws IllegalArgumentException if the index is out of bounds or color is {@code null}
     * @see #setInventory(ChamberState[]) for replacing the entire inventory
     * @see #getInventory() for accessing the entire inventory array
     * @see #getInventory(int) for accessing individual chamber states
     */
    public void setInventory(int index, ChamberState color) {
        if (index < 0 || index >= inventory.length) {
            throw new IllegalArgumentException("Index out of bounds");
        }
        if (color == null) {
            throw new IllegalArgumentException("Color cannot be null");
        }
        inventory[index] = color;
    }

    /**
     * Advances the sorter to the next chamber position based on the current mode. For example, if
     * the current mode is Intake and the current chamber is 0, this will move the servo to the
     * position for chamber 1 in intake mode. If the current chamber is 2, it will wrap around to
     * chamber 0.
     *
     * @see #goToChamber(int) for moving to a specific chamber index
     * @see #goToChamber(ChamberState) for moving to a chamber based on its state
     */
    public void goToChamber() {
        int nextChamber = (currentChamber + 1) % NUM_CHAMBERS;
        int targetRotation;
        switch (currentMode) {
            case Intake:
                targetRotation = intakePositions[nextChamber];
                break;
            case Outtake:
                targetRotation = outtakePositions[nextChamber];
                break;
            default:
                throw new IllegalStateException("Invalid SorterMode: " + currentMode);
        }
        currentChamber = nextChamber;
        sorter.setTargetRotation(targetRotation);
    }

    /**
     * Moves the sorter to the specified chamber index based on the current mode. For example, if
     * the current mode is Outtake and the chamberIndex is 1, this will move the servo to the
     * position for chamber 1 in outtake mode.
     *
     * @param chamberIndex the index of the chamber to move to (0..NUM_CHAMBERS-1)
     * @throws IllegalArgumentException if the chamberIndex is out of bounds
     * @see #goToChamber() for advancing to the next chamber
     * @see #goToChamber(ChamberState) for moving to a chamber based on its state
     */
    public void goToChamber(int chamberIndex) {
        if (chamberIndex < 0 || chamberIndex >= NUM_CHAMBERS) {
            throw new IllegalArgumentException("Chamber index out of bounds");
        }
        int targetRotation;
        switch (currentMode) {
            case Intake:
                targetRotation = intakePositions[chamberIndex];
                break;
            case Outtake:
                targetRotation = outtakePositions[chamberIndex];
                break;
            default:
                throw new IllegalStateException("Invalid SorterMode: " + currentMode);
        }
        currentChamber = chamberIndex;
        sorter.setTargetRotation(targetRotation);
    }

    /**
     * Moves the sorter to the chamber corresponding to the specified {@link ChamberState}. For
     * example, if the current mode is Intake and the chamberState is Green, this will search the
     * inventory for a chamber with state Green and move the servo to that chamber's position in
     * intake mode. If no chamber with the specified state is found, this will throw an exception.
     *
     * @param chamberState the {@link ChamberState} to move to; must not be {@code null}
     * @throws NoSuchElementException if no chamber with the specified state is found
     * @throws IllegalArgumentException if chamberState is {@code null}
     * @see #goToChamber() for advancing to the next chamber
     * @see #goToChamber(int) for moving to a specific chamber index
     */
    public void goToChamber(ChamberState chamberState) {
        if (chamberState == null) {
            throw new IllegalArgumentException("chamberState cannot be null");
        }
        int chamberIndex = -1;
        for (int i = 0; i < inventory.length; i++) {
            if (inventory[i] == chamberState) {
                chamberIndex = i;
                break;
            }
        }
        if (chamberIndex == -1) {
            throw new NoSuchElementException("No chamber with state " + chamberState);
        }
        goToChamber(chamberIndex);
    }

    /**
     * Forces the servo to a specific power output, bypassing the PID controller.
     * Use {@code 0} to hold the servo in place while another mechanism is active.
     *
     * @param power the power to apply (-1.0..1.0)
     */
    public void setPower(double power) {
        sorter.setPower(power);
    }

    /**
     * Returns {@code true} when the servo has reached its target rotation
     * within the default tolerance of 5 degrees.
     */
    public boolean isAtTarget() {
        return sorter.isAtTarget();
    }

    /**
     * Must be called every loop iteration to update the servo position.
     */
    public void update() {
        sorter.update();
    }

    //----------------------------------------------------------------------------------------------
    // Enums
    //----------------------------------------------------------------------------------------------

    public enum SorterMode {
        Intake,
        Outtake
    }

    public enum ChamberState {
        Empty,
        Purple,
        Green
    }
}

