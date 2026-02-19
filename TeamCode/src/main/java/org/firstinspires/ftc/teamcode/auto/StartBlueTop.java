package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.limelight.Limelighter;
import org.firstinspires.ftc.teamcode.MecanumDrive;


public class StartBlueTop extends LinearOpMode {

    MecanumDrive mecanumDrive;
    Limelighter limelight;

    @Override
    public void runOpMode() throws InterruptedException{

        Pose2d beginPose = new Pose2d(new Vector2d(-60.0, 37), Math.toRadians(0));

        mecanumDrive = new MecanumDrive(this.hardwareMap, beginPose);
        limelight = new Limelighter(this.hardwareMap);


        waitForStart();

//        TrajectoryActionBuilder path = mecanumDrive.actionBuilder(beginPose)
//                .afterTime()

    }
}
