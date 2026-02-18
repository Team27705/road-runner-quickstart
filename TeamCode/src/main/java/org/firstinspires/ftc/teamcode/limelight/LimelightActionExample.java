package org.firstinspires.ftc.teamcode.limelight;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Example OpMode demonstrating how to use Limelight Actions with RoadRunner
 * 
 * EXAMPLE 1: Reading ANY AprilTag (including non-motif tags)
 *   - Shows how to detect and get the ID of any AprilTag
 *   - Returns ID, family, and position for any tag detected
 * 
 * EXAMPLE 2: Detecting motif and storing for match duration
 *   - Detect motif once at the start
 *   - Access stored motif throughout the match
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
            // EXAMPLE 1: Read ANY AprilTag and get its ID (even if it's not a motif)
            telemetry.addData("Example 1", "Reading any AprilTag");
            telemetry.update();
            
            // Start Limelight and set pipeline
            limelight.startLimelight(100);
            sleep(100); // Give Limelight time to start up
            limelight.setPipeline(Limelight.Pipelines.APRILTAGGER);
            sleep(100); // Give pipeline time to switch
            
            telemetry.addData("Status", "Limelight started, searching...");
            telemetry.update();
            
            // Create an action to get any AprilTag
            Limelight.GetAprilTagAction getTagAction = limelight.new GetAprilTagAction();
            
            // Run the detection action
            while (opModeIsActive() && getTagAction.run(new com.acmerobotics.dashboard.telemetry.TelemetryPacket())) {
                // Continue running until the action completes
                telemetry.addData("Status", "Searching for AprilTag...");
                
                // Add raw result info for debugging
                com.qualcomm.hardware.limelightvision.LLResult rawResult = limelight.getLatestResult();
                if (rawResult != null) {
                    telemetry.addData("Result Valid", rawResult.isValid());
                    if (rawResult.isValid()) {
                        java.util.List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials = rawResult.getFiducialResults();
                        telemetry.addData("Fiducials Found", fiducials != null ? fiducials.size() : 0);
                    }
                } else {
                    telemetry.addData("Result", "null");
                }
                
                telemetry.update();
                sleep(50); // Give Limelight time to process frames
            }

            // Get the result - this works for ANY AprilTag, not just motifs
            com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult tagResult = getTagAction.getResult();
            
            if (tagResult != null) {
                telemetry.addData("AprilTag Found", "Yes");
                telemetry.addData("AprilTag ID", tagResult.getFiducialId());
                telemetry.addData("AprilTag Family", tagResult.getFamily());
                telemetry.addData("AprilTag X", "%.2f degrees", tagResult.getTargetXDegrees());
                telemetry.addData("AprilTag Y", "%.2f degrees", tagResult.getTargetYDegrees());
            } else {
                telemetry.addData("AprilTag Found", "No");
            }
            telemetry.update();
            
            limelight.closeLimeLight();
            sleep(2000);

            // EXAMPLE 2: Detect motif once at the beginning and store it for the match
            telemetry.addData("Example 2", "Detecting motif for match");
            telemetry.update();
            
            Action detectMotifSequence = new SequentialAction(
                    limelight.startLimelightAction(100),
                    limelight.setPipelineAction(Limelight.Pipelines.APRILTAGGER),
                    limelight.detectArtifactSequenceAction(),
                    limelight.closeLimelightAction()
            );

            // Run the detection sequence
            while (opModeIsActive() && !detectMotifSequence.run(new com.acmerobotics.dashboard.telemetry.TelemetryPacket())) {
                // Continue running until the action completes
                telemetry.addData("Status", "Detecting motif...");
                telemetry.update();
                sleep(50); // Give Limelight time to process frames
            }

            // Now the motif is stored and can be accessed throughout the match
            Limelight.Motif detectedMotif = limelight.getStoredMotif();
            
            if (detectedMotif != null) {
                telemetry.addData("Detected Motif", detectedMotif.name());
                telemetry.addData("Motif ID", detectedMotif.getValue());
                
                // Example: Use the motif to make autonomous decisions
                switch (detectedMotif) {
                    case GPP:
                        telemetry.addData("Action", "Following GPP path");
                        // Execute GPP-specific autonomous routine
                        break;
                    case PGP:
                        telemetry.addData("Action", "Following PGP path");
                        // Execute PGP-specific autonomous routine
                        break;
                    case PPG:
                        telemetry.addData("Action", "Following PPG path");
                        // Execute PPG-specific autonomous routine
                        break;
                }
            } else {
                telemetry.addData("Warning", "No motif detected, using default path");
                // Execute default autonomous routine
            }
            
            telemetry.update();
            
            // Example: Access the stored motif later in the match
            sleep(1000);
            
            // You can check the motif again anytime during the match
            Limelight.Motif storedMotif = limelight.getStoredMotif();
            telemetry.addData("Stored Motif (later)", storedMotif != null ? storedMotif.name() : "None");
            telemetry.update();
            
            sleep(2000);
        }
    }
}
