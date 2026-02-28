package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Bootkicker {
    // Static constants
    private static final double BOOTKICKER_POS_UP = 0.25;
    private static final double BOOTKICKER_POS_DOWN = 0;
    private static final int BOOTKICKER_DELAY = 400; //millis

    // Hardware
    private final Servo bootkicker;

    // State Variables
    private KickerState kickerState;
    private boolean bootkickerCalled;
    private final ElapsedTime bootKickerTimer;

    public Bootkicker(HardwareMap hardwareMap) {
        bootkicker = hardwareMap.get(Servo.class, "bootkicker");
        bootkicker.setDirection(Servo.Direction.REVERSE);

        kickerState = KickerState.Ready;
        bootkickerCalled = false;
        bootKickerTimer = new ElapsedTime();
    }

    /**
     * Initialize the bootkicker to the down position
     */
    public void initialize() {
        bootkicker.setPosition(BOOTKICKER_POS_DOWN);
        kickerState = KickerState.Ready;
    }

    /**
     * Update the bootkicker state machine. Call this regularly in your loop.
     */
    public void update() {
        switch (kickerState) {
            case Ready:
                bootkickerCalled = false;
                break;
            case SendUp:
                bootkickerCalled = true;
                bootkicker.setPosition(BOOTKICKER_POS_UP);
                kickerState = KickerState.SendDown;
                bootKickerTimer.reset();
                break;
            case SendDown:
                if (bootKickerTimer.milliseconds() >= BOOTKICKER_DELAY) {
                    bootkicker.setPosition(BOOTKICKER_POS_DOWN);
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

    /**
     * Trigger a kick action
     */
    public void kick() {
        if (kickerState == KickerState.Ready) {
            kickerState = KickerState.SendUp;
        }
    }

    /**
     * Check if the bootkicker is ready to kick
     */
    public boolean isReady() {
        return kickerState == KickerState.Ready;
    }

    /**
     * Check if the bootkicker has been called (is active)
     */
    public boolean isCalled() {
        return bootkickerCalled;
    }

    /**
     * Get the current kicker state
     */
    public KickerState getState() {
        return kickerState;
    }

    private enum KickerState {
        Ready,
        SendUp,
        SendDown,
        WaitTillDown
    }
}

