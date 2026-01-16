package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "Start Red Top")
public class StartRedTop extends LinearOpMode {

    private Pose2d beginPose = new Pose2d(new Vector2d(-60.0, 37), Math.toRadians(0));


    //hardware

    private MecanumDrive mecanumDrive;

    private Limelight limelight;


    //research InstantFunction


    @Override
    public void runOpMode() throws InterruptedException {

        mecanumDrive = new MecanumDrive(this.hardwareMap, beginPose);
        limelight = new Limelight(this.hardwareMap);



//        mecanumDrive.updatePoseEstimate(); use this to update pose from trajectory to trajectory

        waitForStart();
        //https://learnroadrunner.com/trajectorybuilder-functions.html#splineto-endposition-vector2d-endtangent-double

        TrajectoryActionBuilder test = mecanumDrive.actionBuilder(beginPose)
                .lineToX(-50)
                .waitSeconds(3)
                .splineTo(new Vector2d(-40, 27), Math.toRadians(90));
//                .splineToSplineHeading(new Pose2d(-50, 37, Math.toRadians(0)), Math.toRadians(0))//                .stopAndAdd();

        TrajectoryActionBuilder goToObelisk = mecanumDrive.actionBuilder(beginPose)
                        .splineToSplineHeading(new Pose2d(-34, 0, Math.toRadians(180)), Math.toRadians(45));

        TrajectoryActionBuilder goToShootingZone = mecanumDrive.actionBuilder(new Pose2d(-34, 0, Math.toRadians(180)))
                .turnTo(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(135)), Math.toRadians(0));

        Actions.runBlocking(
                new SequentialAction(
                        test.build()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        goToObelisk,


                )

        );
    }
}
