package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Outtake;


@Autonomous(name = "Start Red Top")
public class StartRedTop extends LinearOpMode {

    private Pose2d beginPose;


    //hardware

    private MecanumDrive mecanumDrive;

    private Limelight limelight;

    private Outtake outtake;


    //research InstantFunction


    @Override
    public void runOpMode() throws InterruptedException {

//        mecanumDrive.updatePoseEstimate(); use this to update pose from trajectory to trajectory


        waitForStart();


        beginPose = new  Pose2d(new Vector2d(-60.0, -37), Math.toRadians(270));

        mecanumDrive = new MecanumDrive(this.hardwareMap, beginPose);
        limelight = new Limelight(this.hardwareMap);
        outtake = new Outtake(this.hardwareMap);
        //https://learnroadrunner.com/trajectorybuilder-functions.html#splineto-endposition-vector2d-endtangent-double

        //at 0 heading positive x is forward, negative x is backwards, positive Y is left negative y right when 0,0 intake facing forward

        TrajectoryActionBuilder test = mecanumDrive.actionBuilder(beginPose)
                .waitSeconds(1)
                .lineToXConstantHeading(10)
                .waitSeconds(1)
                .strafeTo(new Vector2d(10,10));
//                .splineToSplineHeading(new Pose2d(-50, 37, Math.toRadians(0)), Math.toRadians(0))//                .stopAndAdd();

        TrajectoryActionBuilder goToObelisk = mecanumDrive.actionBuilder(beginPose)
                        .splineToSplineHeading(new Pose2d(-34, 0, Math.toRadians(180)), Math.toRadians(45)
                        );

        TrajectoryActionBuilder goToShootingZone = mecanumDrive.actionBuilder(new Pose2d(-34, 0, Math.toRadians(180)))
                .turnTo(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(135)), Math.toRadians(0)
                );

        Actions.runBlocking(
                new SequentialAction(
                        goToObelisk.build(),
                        goToShootingZone.build()
                )
        );

//        Actions.runBlocking(
//                new SequentialAction(
//                        goToObelisk.build()
//                )
//        );

        SequentialAction autoShooting = new SequentialAction(

                //load to match motif, an action inside Spindexer
                //Kick servo action
                //waitSeconds(.5),
                //load next do this 2 more times
                //waitSeconds(1) for the Spindexer

        );

//        Actions.runBlocking(
//                new SequentialAction(
//                        goToObelisk.build(),
//                        //limelight gets motif then updates spindexer motif
//
//                        new ParallelAction(
//                                //spinIndexer w/
//                                goToShootingZone.build(),
//                                outtake.AutoRamp(.8),
//
//                        ),
//                        new SleepAction(10)
//
//                )
//        );
    }
}
