package org.firstinspires.ftc.teamcode.telop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "learning", group = "teachgroup")
public class Learning extends LinearOpMode {
    @Override
    public  void runOpMode() throws InterruptedException{

        Servo myServo = hardwareMap.get(Servo.class, "myServo");

        waitForStart();
        while (opModeIsActive()) {
            myServo.setPosition(.5);
        }
    }
}
