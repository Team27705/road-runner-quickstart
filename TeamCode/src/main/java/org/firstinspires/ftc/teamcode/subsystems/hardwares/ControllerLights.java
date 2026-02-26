package org.firstinspires.ftc.teamcode.subsystems.hardwares;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Controller Lights", group = "Testing")
public class ControllerLights extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            double mappedR = gamepad1.left_trigger;
            double gamepad1Touch1Y = gamepad1.touchpad_finger_1_y;
            double mappedB = gamepad1.right_trigger;

            double mappedG = (gamepad1Touch1Y + 1) / 2; // Map from [-1, 1] to [0, 1]

            if (gamepad1.aWasReleased()) {
                gamepad1.setLedColor(mappedR, mappedG, mappedB, 10000);
            }

            telemetry.addData("Red", mappedR);
            telemetry.addData("Green", mappedG);
            telemetry.addData("Blue", mappedB);
            telemetry.update();
        }
    }
}
