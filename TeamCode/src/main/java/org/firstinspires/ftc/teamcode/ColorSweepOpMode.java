package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@SuppressWarnings("unused")
@TeleOp(name = "Test RGB Indicator - Color Sweep")
public class ColorSweepOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        RGBIndicator led = RGBIndicator.get(hardwareMap, "led");

        while (opModeIsActive()) {
            // sweep through colors, from 0.279 to 0.723 then repeat
            for (double pos = 0.279; pos <= 0.723; pos += 0.001) {
                led.setColor(pos);
                sleep(1);
            }
            for (double pos = 0.723; pos >= 0.279; pos -= 0.001) {
                led.setColor(pos);
                sleep(1);
            }
        }
    }
}
