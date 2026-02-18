package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

@TeleOp(name = "RunLimelight", group = "Testing")
public class RunLimelight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Limelight with the opmode hardwareMap (local variable)
        Limelight limelight = new Limelight(hardwareMap);

        // Start the limelight with a default polling rate
        limelight.startLimelight(60);

        // Set a default pipeline to view (AprilTag detector by default)
        limelight.setPipeline(Limelight.Pipelines.APRILTAGGER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Use the helper to get the first detected AprilTag fiducial (or null)
            LLResultTypes.FiducialResult fr = limelight.getAprilTag();

            if (fr != null) {
                int id = fr.getFiducialId();

                // Map to a Motif if present
                Limelight.Motif matched = null;
                for (Limelight.Motif m : Limelight.Motif.values()) {
                    if (m.getValue() == id) {
                        matched = m;
                        break;
                    }
                }

                // Only show telemetry for matched motifs to avoid spamming the driver station
                if (matched != null) {
                    telemetry.addData("Motif", "%s", matched.name());
                    telemetry.addData("Apriltag ID", id);

                    // Print matched fiducial angles/family to confirm detection details
                    telemetry.addData("Fiducial", String.format(java.util.Locale.US, "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees()));

                    // Update telemetry only when we added fields above (i.e., a useful tag was found)
                    telemetry.update();
                }
            }

            // Sleep a bit to avoid spamming CPU (polling rate is handled by Limelight object)
            sleep(50);
        }

        // Stop the limelight when exiting
        // Note: local variable `limelight` is still accessible here
        limelight.closeLimeLight();
    }
}
