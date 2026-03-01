package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Bootkicker {
    private static final double POS_DOWN = 0.0;
    private static final double POS_UP = 0.25;
    private static final int DELAY_MS = 400;
    private final Servo bootkickerServo;
    private final ElapsedTime timer = new ElapsedTime();
    private State state = State.Ready;
    public Bootkicker(HardwareMap hardwareMap) {
        bootkickerServo = hardwareMap.get(Servo.class, "bootkicker");
        bootkickerServo.setDirection(Servo.Direction.REVERSE);
        bootkickerServo.setPosition(POS_DOWN);
    }

    /**
     * Call once to trigger a single kick cycle. No-ops if already kicking.
     */
    public void kick() {
        if (state != State.Ready) return;
        bootkickerServo.setPosition(POS_UP);
        timer.reset();
        state = State.SendDown;
    }

    /**
     * Must be called every loop iteration.
     */
    public void update() {
        switch (state) {
            case SendDown:
                if (timer.milliseconds() >= DELAY_MS) {
                    bootkickerServo.setPosition(POS_DOWN);
                    timer.reset();
                    state = State.WaitTillDown;
                }
                break;
            case WaitTillDown:
                if (timer.milliseconds() >= DELAY_MS) {
                    state = State.Ready;
                }
                break;
            default:
                break;
        }
    }

    /**
     * Returns true when the kicker is idle and ready to fire again.
     */
    public boolean isReady() {
        return state == State.Ready;
    }

    private enum State {
        Ready,
        SendDown,
        WaitTillDown
    }
}
