package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test RGB Indicator - Color Sweep")
public class ColorSweepOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        
        RGBIndicator led = RGBIndicator.get(hardwareMap, "led");

        while (opModeIsActive()) {
            // sweep through colors, from 0.277 to 0.722 then repeat
            for (double pos = 0.277; pos <= 0.722; pos += 0.001) {
                led.setColor(pos); // map position to enum
                sleep(1);
            }
        }
    }
}
