package org.firstinspires.ftc.teamcode.subsystems.hardwares.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test Class", group = "Testing")
public class ServoTestClass extends LinearOpMode {
    private Servo serville;

    @Config
    public static class positons {
        public static double targetAngle = 0.0;

    }

    @Override
    public void runOpMode() throws InterruptedException {
        serville = hardwareMap.get(Servo.class, "hoodServo");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        serville.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            double x = positons.targetAngle;
            if (gamepad1.dpadUpWasReleased()) {
                serville.setPosition(.5);
            }

            if (gamepad1.dpadDownWasReleased()) {
                serville.setPosition(0);
            }


//            if (gamepad1.dpadDownWasReleased()) {
//
//            } else if (gamepad1.dpadUpWasReleased()) {
//                serville.setPosition(0.5);
//            }
        }
    }

}
