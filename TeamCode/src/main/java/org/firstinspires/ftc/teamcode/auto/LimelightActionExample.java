package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Example OpMode demonstrating how to use Limelight Actions with RoadRunner
 */
@Autonomous(name = "Limelight Action Example", group = "Testing")
public class LimelightActionExample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        Pose2d beginPose = new Pose2d(new Vector2d(-60.0, 37), Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Limelight limelight = new Limelight(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example 1: Simple sequence of Limelight actions
            Action limelightSequence = new SequentialAction(
                    limelight.startLimelightAction(100),
                    limelight.setPipelineAction(Limelight.Pipelines.APRILTAGGER),
                    limelight.detectArtifactSequenceAction(),
                    limelight.closeLimelightAction()
            );

            // Run the action sequence
            com.acmerobotics.roadrunner.Actions.runBlocking(limelightSequence);

            telemetry.addData("Status", "Limelight actions completed");
            telemetry.update();

            // Example 2: Using Limelight actions with movement
            Action complexSequence = drive.actionBuilder(beginPose)
                    .afterTime(0, limelight.startLimelightAction())
                    .lineToX(-50)
                    .afterTime(0, limelight.setPipelineAction(Limelight.Pipelines.APRILTAGGER))
                    .waitSeconds(1)
                    .afterTime(0, limelight.getAprilTagAction())
                    .lineToX(-40)
                    .afterTime(0, limelight.closeLimelightAction())
                    .build();

            com.acmerobotics.roadrunner.Actions.runBlocking(complexSequence);

            telemetry.addData("Status", "Complex sequence completed");
            telemetry.update();
        }
    }
}
