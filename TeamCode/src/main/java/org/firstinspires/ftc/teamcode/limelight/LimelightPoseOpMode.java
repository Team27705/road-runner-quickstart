package org.firstinspires.ftc.teamcode.limelight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * OpMode that reads an AprilTag with the Limelight and publishes a Pose2d to
 * the FTCDashboard representing the robot position computed from the
 * AprilTag -> Limelight vector.
 *
 * Notes:
 * - This OpMode assumes the AprilTag fixed position on the field is known.
 * - Provide the AprilTag world pose (x, y, heading) and tag height in inches.
 * - The camera height must be supplied to compute range from vertical angle.
 */
@Autonomous(name = "Limelight Pose Publisher", group = "Limelight")
public class LimelightPoseOpMode extends LinearOpMode {

    // Example world pose of the AprilTag (field coordinates in inches)
    // Modify to match your field layout and which AprilTag you're reading.
    private static final double APRILTAG_X_IN = 72.0; // inches
    private static final double APRILTAG_Y_IN = 36.0; // inches
    private static final double APRILTAG_HEADING_RAD = 0.0; // radians (optional)
    private static final double APRILTAG_HEIGHT_IN = 12.0; // tag center height

    // Camera height above the ground in inches
    private static final double CAMERA_HEIGHT_IN = 6.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight limelight = new Limelight(hardwareMap);

        // Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Start limelight and set april tag pipeline
        limelight.startLimelight(50);
        sleep(100);
        limelight.setPipeline(Limelight.Pipelines.APRILTAGGER);
        sleep(100);

        waitForStart();

        while (opModeIsActive()) {
            // Get vector from AprilTag to Limelight (in inches). This represents
            // where the camera is relative to the AprilTag. Use the no-switch
            // variant because we've already started the Limelight and set the
            // APRILTAGGER pipeline above.
            Vector2d tagToCam = limelight.getLimelightVectorFromLatestObservation(APRILTAG_HEIGHT_IN, CAMERA_HEIGHT_IN);

            TelemetryPacket packet = new TelemetryPacket();

            if (tagToCam != null) {
                // Compute robot pose: AprilTag world pose + tag->cam vector
                // We assume the AprilTag frame's +x is forward and +y is left, matching
                // our Vector2d convention; if your field uses different axes, adjust.
                double robotX = APRILTAG_X_IN + tagToCam.x; // forward
                double robotY = APRILTAG_Y_IN + tagToCam.y; // left
                double robotHeading = APRILTAG_HEADING_RAD; // This simple example does not compute heading

                Pose2d robotPose = new Pose2d(robotX, robotY, robotHeading);

                packet.put("robotPose", robotPose);
                packet.put("robotX_in", robotX);
                packet.put("robotY_in", robotY);
                packet.put("robotHeading_rad", robotHeading);
                packet.put("apriltagDetected", true);

                // Draw robot on the field overlay and mark the AprilTag
                Canvas field = packet.fieldOverlay();
                // Draw AprilTag position as a small filled circle
                field.setStrokeWidth(1);
                field.setStroke("red");
                field.fillCircle((float) APRILTAG_X_IN, (float) APRILTAG_Y_IN, 4);
                // Draw robot using the existing Drawing helper (drawRobot expects Pose2d)
                Drawing.drawRobot(field, robotPose);
            } else {
                packet.put("apriltagDetected", false);
            }

            // Send packet to dashboard
            dashboard.sendTelemetryPacket(packet);

            // Also send to OpMode telemetry for debugging
            if (tagToCam != null) {
                telemetry.addData("Pose", "x=%.2f in, y=%.2f in", tagToCam.x, tagToCam.y);
            } else {
                telemetry.addData("Pose", "No AprilTag");
            }
            telemetry.update();

            sleep(200);
        }

        // Clean up
        limelight.closeLimeLight();
    }
}
