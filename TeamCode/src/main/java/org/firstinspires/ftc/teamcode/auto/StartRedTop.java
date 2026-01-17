package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "Start Red Top")
public class StartRedTop extends LinearOpMode {

    //research InstantFunction


    @Override
    public void runOpMode() throws InterruptedException {

//        mecanumDrive.updatePoseEstimate(); use this to update pose from trajectory to trajectory


        waitForStart();


        Pose2d beginPose = new Pose2d(new Vector2d(-60.0, -37), Math.toRadians(0));

        MecanumDrive mecanumDrive = new MecanumDrive(this.hardwareMap, beginPose);
        // removed unused Limelight and Outtake instances; re-add when using their functionality
        // Limelight limelight = new Limelight(this.hardwareMap);
        // Outtake outtake = new Outtake(this.hardwareMap);
        //https://learnroadrunner.com/trajectorybuilder-functions.html#splineto-endposition-vector2d-endtangent-double

        //at 0 heading positive x is forward, negative x is backwards, positive Y is left negative y right when 0,0 intake facing forward

        TrajectoryActionBuilder goToObelisk = mecanumDrive.actionBuilder(beginPose)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-34, 0, Math.toRadians(180)), Math.toRadians(10));

        TrajectoryActionBuilder goToShootingZone0 = mecanumDrive
                .actionBuilder(new Pose2d(-34, 0, Math.toRadians(180)))
                .turnTo(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-10, -10), Math.toRadians(135))
                .waitSeconds(1);

        TrajectoryActionBuilder goToBallSet1 = mecanumDrive
                .actionBuilder(new Pose2d(-10, -10, Math.toRadians(135)))
                .splineToLinearHeading(new Pose2d(-11, -30, Math.toRadians(90)), Math.toRadians(20));

        TrajectoryActionBuilder intakeBallSet1 = mecanumDrive
                .actionBuilder(new Pose2d(-11, -30, Math.toRadians(90)))
                .lineToY(50)
                .turnTo(Math.toRadians(180));

        TrajectoryActionBuilder goToShootingZone1 = mecanumDrive
                .actionBuilder(new Pose2d(-11, -50, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(135)), Math.toRadians(-10))
                .waitSeconds(1);

        TrajectoryActionBuilder goToBallSet2 = mecanumDrive
                .actionBuilder(new Pose2d(-10, -10, Math.toRadians(135)))
                .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(90)), Math.toRadians(-20));

        TrajectoryActionBuilder intakeBallSet2 = mecanumDrive
                .actionBuilder(new Pose2d(12, -30, Math.toRadians(90)))
                .lineToY(50);

        TrajectoryActionBuilder goToShootingZone2 = mecanumDrive
                .actionBuilder(new Pose2d(12, -50, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(135)), Math.toRadians(30))
                .waitSeconds(1);

        TrajectoryActionBuilder moveForward = mecanumDrive.actionBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .lineToXConstantHeading(10);

        Actions.runBlocking(
                new SequentialAction(
                        goToObelisk.build(),
                        goToShootingZone0.build(),
                        goToBallSet1.build(),
                        intakeBallSet1.build(),
                        goToShootingZone1.build(),
                        goToBallSet2.build()
                )
        );

        Actions.runBlocking(
                new SequentialAction(

                )
        );

//        Actions.runBlocking(
//                new SequentialAction(
//                        goToObelisk.build()
//                )
//        );

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
