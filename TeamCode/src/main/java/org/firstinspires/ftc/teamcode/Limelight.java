package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        // Use lowercase device name "limelight" to match the standard configuration and samples
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

    // Backwards-compatible no-arg method. If Blocks-style static telemetry is available it will be used.
    public void updateTelemetry() {
        if (telemetry != null) {
            updateTelemetry(telemetry);
        }
    }

    // Preferred method: pass in the OpMode telemetry object so output goes to the current OpMode.
    public void updateTelemetry(@NonNull Telemetry t) {
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) {
            t.addData("Limelight", "No valid target found");
        } else {
            t.addData("Limelight", "Valid target found");
            double tx = llResult.getTx(); // Horizontal offset from crosshair to target
            double ty = llResult.getTy(); // Vertical offset from crosshair to target
            double ta = llResult.getTa(); // Target area (0% of image to 100

            t.addData("LLTargetX", tx);
            t.addData("LLTargetY", ty);
            t.addData("LLTargetA", ta);
        }
    }

    // Expose the raw latest result so OpModes can inspect fiducial/apriltag outputs, etc.
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public final class getArtifactSequence {
        public void run(@NonNull TelemetryPacket packet) {
            startLimelight();
            Pipelines pipeline = getPipeline();
            setPipeline(Pipelines.APRILTAGGER);
            updateTelemetry();
            setPipeline(pipeline);
            closeLimeLight();
        }
    }

}
