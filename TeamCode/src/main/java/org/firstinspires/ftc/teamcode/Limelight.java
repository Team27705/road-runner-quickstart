package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    //https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming

    public enum Pipelines {
        APRILTAGGER(0),
        PURPLER(1),
        GREENER(2);

        private final int value;

        Pipelines(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    private final Limelight3A limelight;

    private int limelightMode; // 0 is to track AprilTags, 1 is used to track for balls and the colors

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void startLimelight() {
        startLimelight(100);
    }

    public void startLimelight(int poll_rate) {
        setLimelightPollingRate(poll_rate);
        limelight.start();
    }

    public void closeLimeLight() {
        limelight.close();
    }

    public void setLimelightPollingRate(int rate) {
        limelight.setPollRateHz(rate);
    }

    public void setPipeline(Pipelines pipeline) {
        // Use the API provided by the Limelight3A sample: pipelineSwitch(index)
        limelight.pipelineSwitch(pipeline.getValue());
    }

    public Pipelines getPipeline() {
        // Read the pipeline index via the status object (matches sample code)
        LLStatus status = limelight.getStatus();
        if (status == null) return null;
        int index = status.getPipelineIndex();
        for (Pipelines pipeline : Pipelines.values()) {
            if (pipeline.getValue() == index) {
                return pipeline;
            }
        }
        return null; // or throw an exception if preferred
    }

    //**
    // Change pipeline depending on weather or not you need to get distance from an AprilTag or the ball
    // Use
    // *//

    public void updateTelemetry() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) {
            telemetry.addData("Limelight", "No valid target found");
            return;
        } else {
            telemetry.addData("Limelight", "Valid target found");
            double tx = llResult.getTx(); // Horizontal offset from crosshair to target
            double ty = llResult.getTy(); // Vertical offset from crosshair to target
            double ta = llResult.getTa(); // Target area (0% of image to 100

            telemetry.addData("LLTargetX", tx);
            telemetry.addData("LLTargetY", ty);
            telemetry.addData("LLTargetA", ta);
        }
    }

    public final class setArtifactSequence(int sequence) {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if () {
                startLimelight();
                List<FiducialResult> fiducials = result.getFiducialResults();
                for (FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId(); // The ID number of the fiducial
                    double x = detection.getTargetXDegrees(); // Where it is (left-right)
                    double y = detection.getTargetYDegrees(); // Where it is (up-down)
                    double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getY();
                    telemetry.addData("Fiducial " + id, "is " + distance + " meters away");
                }
            }
            //robot should move/already be in position to shooting position
            // then rotate till it sees april tag and feeds it to spindexer
            artifactSequence =;
            return
        }
    }

}
