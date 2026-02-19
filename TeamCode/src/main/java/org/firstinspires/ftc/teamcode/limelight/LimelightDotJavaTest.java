package org.firstinspires.ftc.teamcode.limelight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.limelight.Limelighter;

@Autonomous(name = "Limelight.java Test", group = "Testing")
public class LimelightDotJavaTest extends LinearOpMode {
    @Override
    public  void runOpMode() throws InterruptedException {
        // Initialize Limelight with the opmode hardwareMap (local variable)
        Limelighter limelight = new Limelighter(hardwareMap);

        IMU imu;

        waitForStart();

        while (opModeIsActive()) {
            // set up internalIMU for testing
            imu = hardwareMap.get(IMU.class, "internalIMU");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )));
            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            telemetry.addData("Limelight Status", "Initialized");
            telemetry.update();

            // Use the helper to get the first detected AprilTag fiducial (or null)
            Pose2d fr = null;
            try {
                fr = limelight.getRobotPoseFromAprilTag(robotYaw);
            } catch (Exception e) {
                telemetry.addData("Exception Thrown", e.getMessage());
                telemetry.update();
            }

            // send pose to ftc dashboard
            if (fr != null) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), fr);

                // Also draw the MT2-calculated position explicitly as a filled circle
                packet.fieldOverlay().setStroke("red");
                packet.fieldOverlay().fillCircle(fr.position.x, fr.position.y, 4);

                // Put the MT2 position into the telemetry packet for easy inspection
                packet.put("mt2_x_in", fr.position.x);
                packet.put("mt2_y_in", fr.position.y);

                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            // Sleep a bit to avoid spamming CPU (polling rate is handled by Limelight object)
            sleep(100);
        }

        // Stop the limelight when exiting
        // Note: local variable `limelight` is still accessible here
        limelight.stopLimelight();
    }
}
