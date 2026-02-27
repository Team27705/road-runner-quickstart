package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;


@Autonomous(name = "Start Red Top")
public class StartRedTop extends LinearOpMode {
    //Trajectories
    private Pose2d beginPose;
    private TrajectoryActionBuilder goToObelisk;
    private TrajectoryActionBuilder goToShootingZone0;
    private TrajectoryActionBuilder goToBallSet1;
    private TrajectoryActionBuilder intakeBallSet1;
    private TrajectoryActionBuilder goToShootingZone1;
    private TrajectoryActionBuilder goToBallSet2;
    private TrajectoryActionBuilder intakeBallSet2;
    private TrajectoryActionBuilder goToShootingZone2;
    private TrajectoryActionBuilder moveForward;
    private TrajectoryActionBuilder turnInPlace;

    //Hardware
    private MecanumDrive mecanumDrive;
    private Spindexer spindexer;
    private Intake intake;

    //Flag Variables
    private int autonStep;
    private boolean isInitialized = false;
    public void buildPaths () {
        goToObelisk = mecanumDrive.actionBuilder(beginPose)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-34, 0, Math.toRadians(180)), Math.toRadians(10));

        goToShootingZone0 = mecanumDrive
                .actionBuilder(new Pose2d(-34, 0, Math.toRadians(180)))
                .turnTo(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-10, -10), Math.toRadians(135))
                .waitSeconds(1);

        goToBallSet1 = mecanumDrive
                .actionBuilder(new Pose2d(-10, -10, Math.toRadians(135)))
                .splineToLinearHeading(new Pose2d(-11, -30, Math.toRadians(90)), Math.toRadians(20));

        intakeBallSet1 = mecanumDrive
                .actionBuilder(new Pose2d(-11, -30, Math.toRadians(90)))
                .lineToY(50)
                .turnTo(Math.toRadians(180));

        goToShootingZone1 = mecanumDrive
                .actionBuilder(new Pose2d(-11, -50, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(135)), Math.toRadians(-10))
                .waitSeconds(1);

        goToBallSet2 = mecanumDrive
                .actionBuilder(new Pose2d(-10, -10, Math.toRadians(135)))
                .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(90)), Math.toRadians(-20));

        intakeBallSet2 = mecanumDrive
                .actionBuilder(new Pose2d(12, -30, Math.toRadians(90)))
                .lineToY(50);

        goToShootingZone2 = mecanumDrive
                .actionBuilder(new Pose2d(12, -50, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(135)), Math.toRadians(30))
                .waitSeconds(1);

        moveForward = mecanumDrive.actionBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .lineToXConstantHeading(10);

        turnInPlace = mecanumDrive.actionBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .turn(Math.toRadians(360));
    }


    @Override
    public void runOpMode() throws InterruptedException {

//        mecanumDrive.updatePoseEstimate(); use this to update pose from trajectory to trajectory


        waitForStart();

        beginPose = new Pose2d(new Vector2d(-60.0, -37), Math.toRadians(0));

        mecanumDrive = new MecanumDrive(this.hardwareMap, beginPose);
        spindexer = new Spindexer(this.hardwareMap, true);
        intake = new Intake(this.hardwareMap);
        // removed unused Limelight and Outtake instances; re-add when using their functionality
        // Limelight limelight = new Limelight(this.hardwareMap);
        // Outtake outtake = new Outtake(this.hardwareMap);
        //https://learnroadrunner.com/trajectorybuilder-functions.html#splineto-endposition-vector2d-endtangent-double

        //at 0 heading positive x is forward, negative x is backwards, positive Y is left negative y right when 0,0 intake facing forward



        while (!isStopRequested()) {
            if (!isInitialized) {
                buildPaths();
                autonStep = 0;
                isInitialized = false;
            }
            else return;

            spindexer.update();


            switch (autonStep) {
                case -1:
                    break;
                case 0:
                    Action.run
                    break;
            }
        }


        Actions.runBlocking(
                new SequentialAction(
                        turnInPlace.build()
//                        goToObelisk.build()
//                        goToShootingZone0.build(),
//                        goToBallSet1.build(),
//                        intakeBallSet1.build(),
//                        goToShootingZone1.build(),
//                        goToBallSet2.build()
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
