package org.firstinspires.ftc.teamcode.limelight;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Drawing;

@Autonomous(name = "Very Sour Lime + Blinding Light", group = "limelight")
public class VerySourLimeBlindingLight extends LinearOpMode {
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        IMU imu = hardwareMap.get(IMU.class, "internalIMU");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);

        // Bridge opmode telemetry to FTC Dashboard so telemetry goes to both places
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // throttle dashboard packet sends to avoid saturating network/phone
        final long DASHBOARD_SEND_INTERVAL_MS = 100; // ~10 Hz
        long lastDashboardSend = 0;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("Limelight", "Target(s) Found!");
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            // shove orientation data to limelight
            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
            limelight.updateRobotOrientation(robotYaw);
            if (result != null && result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");

                    // Convert Limelight Pose3D (assumed meters) to Roadrunner Pose2d (inches)
                    double METERS_TO_INCHES = 39.3700787;
                    double x_in = x * METERS_TO_INCHES;
                    double y_in = y * METERS_TO_INCHES;

                    // Use IMU yaw (already pushed to Limelight) as heading for visualization
                    // Note: imu.getRobotYawPitchRollAngles().getYaw() returns radians already

                    // Build a Roadrunner Pose2d. Adjust axes if the overlay appears flipped in testing.
                    Pose2d rrPose = new Pose2d(new Vector2d(x_in, y_in), robotYaw);

                    // Send to FTC Dashboard field overlay (throttled)
                    long now = System.currentTimeMillis();
                    if (now - lastDashboardSend >= DASHBOARD_SEND_INTERVAL_MS) {
                        lastDashboardSend = now;
                        try {
                            TelemetryPacket packet = new TelemetryPacket();
                            packet.fieldOverlay().setStroke("#3F51B5");
                            Drawing.drawRobot(packet.fieldOverlay(), rrPose);

                            // Also draw the MT2-calculated position explicitly as a filled circle
                            packet.fieldOverlay().setStroke("red");
                            packet.fieldOverlay().fillCircle((float)x_in, (float)y_in, 4);

                            // Put the MT2 position into the telemetry packet for easy inspection
                            packet.put("mt2_x_in", x_in);
                            packet.put("mt2_y_in", y_in);

                            FtcDashboard.getInstance().sendTelemetryPacket(packet);
                        } catch (Exception e) {
                            // don't let dashboard problems crash the opmode; log to telemetry
                            telemetry.addData("DashboardErr", e.getMessage());
                        }
                    }

                    // Also publish the converted values to opmode telemetry for debugging
                    telemetry.addData("MT2 (in)", "x=%.2f, y=%.2f, headingDeg=%.1f", x_in, y_in, Math.toDegrees(robotYaw));
                }
            }
            telemetry.update();
        }
    }
}
