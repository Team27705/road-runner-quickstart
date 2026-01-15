package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "Start Red Top")
public class StartRedTop extends LinearOpMode {

    Pose2d beginPose = new Pose2d(new Vector2d(-60.0, 37), Math.toRadians(0));


    //hardware

    MecanumDrive mecanumDrive = new MecanumDrive(this.hardwareMap, beginPose);


    //research InstantFunction


    @Override
    public void runOpMode() throws InterruptedException {
        Action path = mecanumDrive.actionBuilder(beginPose)
//                .stopAndAdd()
                .build();
    }
}
