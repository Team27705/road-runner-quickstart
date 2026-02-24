package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "SpinInPlace")
public class SpinInPlace extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Pose2d beginPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(0));
        MecanumDrive mecanumDrive = new MecanumDrive(this.hardwareMap, beginPose);

        TrajectoryActionBuilder turnInPlace = mecanumDrive.actionBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .turn(Math.toRadians(360));

        Actions.runBlocking(
                new SequentialAction(
                        turnInPlace.build()
                )
        );


    }
}
