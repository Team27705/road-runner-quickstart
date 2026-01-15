package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StartBlueTop extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        Pose2d beginPose = new Pose2d(new Vector2d(-43.0, ), Math.toRadians(0));

        waitForStart();

        Action path = drive.actionBuilder(beginPose);
    }
}
