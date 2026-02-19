package org.firstinspires.ftc.teamcode.limelight;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Drawing;

@Autonomous(name = "Very Sour Lime + Blinding Light", group = "limelight")
public class VerySourLimeBlindingLight extends LinearOpMode {
    private static final double METERS_TO_INCHES = 39.3700787;
    // Toggle these if your axes are flipped or swapped.
    private static final boolean MT2_SWAP_XY = false;
    private static final double MT2_X_SIGN = 1.0;
    private static final double MT2_Y_SIGN = 1.0;

    Limelight3A limelight;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "internalIMU");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);

        waitForStart();

        // Bridge opmode telemetry to FTC Dashboard so telemetry goes to both places
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // throttle dashboard packet sends to avoid saturating network/phone
        final long DASHBOARD_SEND_INTERVAL_MS = 100; // ~10 Hz
        long lastDashboardSend = 0;
        double lastXIn = 0.0;
        double lastYIn = 0.0;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            // shove orientation data to limelight
            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            limelight.updateRobotOrientation(Math.toDegrees(robotYaw));


            if (result != null && result.isValid()) {
                telemetry.addData("Limelight", "Target(s) Found!");
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double rawX = botpose_mt2.getPosition().x;
                    double rawY = botpose_mt2.getPosition().y;

                    double mt2X = MT2_SWAP_XY ? rawY : rawX;
                    double mt2Y = MT2_SWAP_XY ? rawX : rawY;

                    double x = MT2_X_SIGN * mt2X;
                    double y = MT2_Y_SIGN * mt2Y;

                    telemetry.addData("MT2 Raw (m)", "x=%.3f, y=%.3f", rawX, rawY);
                    telemetry.addData("MT2 Mapped (m)", "x=%.3f, y=%.3f", x, y);
                    telemetry.addData("MT2 Location:", "(%.3f, %.3f)", x, y);

                    // Convert Limelight Pose3D (meters) to Roadrunner Pose2d (inches)
                    double x_in = x * METERS_TO_INCHES;
                    double y_in = y * METERS_TO_INCHES;
                    lastXIn = x_in;
                    lastYIn = y_in;

                    // Also publish the converted values to opmode telemetry for debugging
                    telemetry.addData("MT2 (in)", "x=%.2f, y=%.2f, headingDeg=%.1f", x_in, y_in, Math.toDegrees(robotYaw));
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            // Always update the field overlay heading using the latest IMU yaw.
            long now = System.currentTimeMillis();
            if (now - lastDashboardSend >= DASHBOARD_SEND_INTERVAL_MS) {
                lastDashboardSend = now;
                try {
                    Pose2d rrPose = new Pose2d(new Vector2d(lastXIn, lastYIn), robotYaw);
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.fieldOverlay().setStroke("#3F51B5");
                    Drawing.drawRobot(packet.fieldOverlay(), rrPose);

                    // Also draw the MT2-calculated position explicitly as a filled circle
                    packet.fieldOverlay().setStroke("red");
                    packet.fieldOverlay().fillCircle((float) lastXIn, (float) lastYIn, 4);

                    // Put the MT2 position into the telemetry packet for easy inspection
                    packet.put("mt2_x_in", lastXIn);
                    packet.put("mt2_y_in", lastYIn);

                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                } catch (Exception e) {
                    // don't let dashboard problems crash the opmode; log to telemetry
                    telemetry.addData("DashboardErr", e.getMessage());
                }
            }
            telemetry.update();
        }
        limelight.stop();
    }
}
