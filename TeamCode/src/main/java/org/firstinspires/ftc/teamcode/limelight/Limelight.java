package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

public class Limelight {
    // https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming

    private final Limelight3A limelight;
    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

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

    private Pipelines limelightMode = Pipelines.APRILTAGGER;

    public enum Motif {
        GPP(21),
        PGP(22),
        PPG(23);

        private final int value;

        Motif(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    private HashMap motifToInventory = new HashMap<Motif, String[]>() {{
        put(Motif.GPP, new String[]{"G", "P", "P"});
        put(Motif.PGP, new String[]{"P", "G", "P"});
        put(Motif.PPG, new String[]{"P", "P", "G"});
    }};

    public Motif detectedMotif = null;

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
        if (status == null)
            return null;
        int index = status.getPipelineIndex();
        for (Pipelines pipeline : Pipelines.values()) {
            if (pipeline.getValue() == index) {
                return pipeline;
            }
        }
        return null; // or throw an exception if preferred
    }

    // Expose the raw latest result so OpModes can inspect fiducial/apriltag
    // outputs, etc.
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /**
     * Return the first detected AprilTag fiducial result, or null if none.
     * This method will temporarily switch the pipeline to APRILTAGGER, read the
     * latest result, then restore the previous pipeline and close the Limelight.
     */
    public LLResultTypes.FiducialResult getAprilTag() {
        // Start the limelight (harmless if already started)
        startLimelight();

        // Remember current pipeline and switch to AprilTag detector
        Pipelines prior = getPipeline();
        setPipeline(Pipelines.APRILTAGGER);

        // Read latest result
        LLResult result = getLatestResult();
        LLResultTypes.FiducialResult found = null;

        if (result != null && result.isValid()) {
            java.util.List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                // Return the first fiducial found
                found = fiducials.get(0);
            }
        }

        // Restore prior pipeline
        if (prior != null)
            setPipeline(prior);

        // Close limelight to conserve resources
        closeLimeLight();

        return found;
    }
}