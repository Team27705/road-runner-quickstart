package org.firstinspires.ftc.teamcode.subsystems.hardwares;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@TeleOp(name = "Odo Visualizer", group = "Testing")
public class OdoVisualizer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        TelemetryPacket tPacket = new TelemetryPacket();

        waitForStart();

        // get pose estimate and display on ftc dashboard telemetry
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d poseEstimate = drive.localizer.getPose();
            tPacket.put("x", poseEstimate.position.x);
            tPacket.put("y", poseEstimate.position.y);
            tPacket.put("heading", poseEstimate.heading.toDouble());
        }
    }
}
