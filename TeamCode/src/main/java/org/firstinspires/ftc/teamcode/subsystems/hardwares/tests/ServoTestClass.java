package org.firstinspires.ftc.teamcode.subsystems.hardwares.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test Class", group = "Testing")
public class ServoTestClass extends LinearOpMode {
    Servo serville;

    @Override
    public void runOpMode() throws InterruptedException {
        serville = hardwareMap.get(Servo.class, "serville");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpadDownWasReleased()) {
                serville.setPosition(0);
            } else if (gamepad1.dpadUpWasReleased()) {
                serville.setPosition(0.5);
            }
        }
    }

}
