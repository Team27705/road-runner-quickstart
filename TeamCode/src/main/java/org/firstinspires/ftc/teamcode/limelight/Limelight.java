package org.firstinspires.ftc.teamcode.limelight;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Limelight {
    // https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming

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

    // Stores the motif detected at the start of auto for the entire match duration
    private Motif storedMotif = null;

    public Limelight(HardwareMap hardwareMap) {
        // Use lowercase device name "limelight" to match the standard configuration and
        // samples
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    /**
     * Get the stored motif that was detected at the start of auto
     * 
     * @return the stored Motif, or null if not yet detected
     */
    public Motif getStoredMotif() {
        return storedMotif;
    }

    /**
     * Manually set the stored motif (useful for testing)
     * 
     * @param motif the motif to store
     */
    public void setStoredMotif(Motif motif) {
        this.storedMotif = motif;
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

    // **
    // Change pipeline depending on weather or not you need to get distance from an
    // AprilTag or the ball
    // Use
    // *//

    // Backwards-compatible no-arg method. If Blocks-style static telemetry is
    // available it will be used.
    public void updateTelemetry() {
        if (telemetry != null) {
            updateTelemetry(telemetry);
        }
    }

    // Preferred method: pass in the OpMode telemetry object so output goes to the
    // current OpMode.
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

    /**
     * Read the first AprilTag fiducial and convert the reading into a 2D vector
     * pointing from the AprilTag to the Limelight (camera) in inches.
     *
     * Coordinates / conventions:
     * - Returned Vector2d uses the project's RoadRunner convention where
     *   x = forward distance (positive forward), y = left offset (positive left).
     * - The calculation uses the fiducial result's horizontal and vertical target
     *   angles (degrees) to estimate range using simple trigonometry:
     *     distance = (tagHeight - cameraHeight) / tan(verticalAngle)
     *   and lateral offset = distance * tan(horizontalAngle).
     * - Horizontal angle (tx) from Limelight is typically positive to the right;
     *   to convert to RoadRunner's positive-left Y we negate the lateral component.
     *
     * Assumptions & failure modes:
     * - Caller must supply tag and camera heights in inches (same units).
     * - If no valid fiducial is found or the geometry is degenerate (vertical
     *   angle near zero), the method returns null.
     * - This is an approximation that assumes the tag and camera are roughly
     *   level and that the vertical angle provided is the elevation angle to the
     *   tag center.
     *
     * @param tagHeightInches AprilTag center height above the ground (inches)
     * @param cameraHeightInches Limelight/camera center height above the ground (inches)
     * @return Vector2d from AprilTag -> Limelight in inches (x forward, y left), or null
     */
    public Vector2d getLimelightVectorFromAprilTag(double tagHeightInches, double cameraHeightInches) {
        // Start and switch to APRILTAGGER pipeline (matches other helper methods)
        startLimelight();
        Pipelines prior = getPipeline();
        setPipeline(Pipelines.APRILTAGGER);

        LLResult llResult = getLatestResult();
        Vector2d vec = null;

        if (llResult != null && llResult.isValid()) {
            java.util.List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                LLResultTypes.FiducialResult fr = fiducials.get(0);
                double txDeg = fr.getTargetXDegrees();
                double tyDeg = fr.getTargetYDegrees();

                // Convert to radians
                double tx = Math.toRadians(txDeg);
                double ty = Math.toRadians(tyDeg);

                // Avoid division by zero for near-horizontal sightings
                double tanTy = Math.tan(ty);
                if (Math.abs(tanTy) > 1e-6) {
                    // Estimate forward range from vertical angle and known height difference
                    double range = (tagHeightInches - cameraHeightInches) / tanTy;

                    // Lateral offset: positive to the right from Limelight API; convert
                    // to RoadRunner positive-left by negating.
                    double lateral = range * Math.tan(tx);
                    double rrY = -lateral; // left

                    vec = new Vector2d(range, rrY);
                }
            }
        }

        // Restore pipeline and close limelight (keep behavior consistent)
        if (prior != null) setPipeline(prior);
        closeLimeLight();

        return vec;
    }

    /**
     * Read the latest Limelight observation (no pipeline switching or start/stop)
     * and convert the first fiducial into a 2D vector from AprilTag -> Limelight.
     * This variant is intended for high-frequency loops where the caller has
     * already started the Limelight and selected the desired pipeline.
     *
     * @see #getLimelightVectorFromAprilTag(double,double)
     */
    public Vector2d getLimelightVectorFromLatestObservation(double tagHeightInches, double cameraHeightInches) {
        LLResult llResult = getLatestResult();
        if (llResult == null || !llResult.isValid()) return null;

        java.util.List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        LLResultTypes.FiducialResult fr = fiducials.get(0);
        double txDeg = fr.getTargetXDegrees();
        double tyDeg = fr.getTargetYDegrees();

        double tx = Math.toRadians(txDeg);
        double ty = Math.toRadians(tyDeg);
        double tanTy = Math.tan(ty);
        if (Math.abs(tanTy) <= 1e-6) return null;

        double range = (tagHeightInches - cameraHeightInches) / tanTy;
        double lateral = range * Math.tan(tx);
        double rrY = -lateral;

        return new Vector2d(range, rrY);
    }

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

    public final class ArtifactSequence {

        public Motif detectedMotif = null;

        // Read AprilTags from the Limelight, match the tag ID to a Motif, set
        // detectedMotif, and return it.
        // Returns null if no matching AprilTag is found or no valid result is
        // available.
        public Motif update(@NonNull TelemetryPacket packet) {
            // Start the limelight (harmless if already started in most use-cases)
            startLimelight();

            // Remember current pipeline and switch to AprilTag detector
            Pipelines prior = getPipeline();
            setPipeline(Pipelines.APRILTAGGER);

            // Give the limelight one poll cycle to update (poll rate controls actual
            // timing). We attempt to use
            // the latest available result immediately.
            LLResult result = getLatestResult();
            Motif found = null;

            if (result != null && result.isValid()) {
                java.util.List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    // Iterate and pick the first fiducial that maps to a known Motif
                    for (LLResultTypes.FiducialResult fr : fiducials) {
                        int id = fr.getFiducialId();
                        Motif m = motifFromId(id);
                        if (m != null) {
                            found = m;
                            // Optionally publish info to the dashboard packet
                            packet.put("apriltag_id", id);
                            packet.put("detected_motif", m.name());
                            break;
                        }
                    }
                } else {
                    packet.put("apriltag", "none");
                }
            } else {
                packet.put("apriltag_result", "invalid");
            }

            // Restore prior pipeline
            if (prior != null)
                setPipeline(prior);

            // Close limelight to conserve resources (matches previous behavior)
            closeLimeLight();

            detectedMotif = found;
            return found;
        }

        // Helper to map AprilTag ID to Motif enum. Returns null if no mapping exists.
        private Motif motifFromId(int id) {
            for (Motif m : Motif.values()) {
                if (m.getValue() == id)
                    return m;
            }
            return null;
        }

    }

    // RoadRunner Actions for Limelight functionality

    /**
     * Action to start the Limelight with a specified polling rate
     * Note: This Action is designed for single use. Create a new instance for each
     * use.
     */
    public class StartLimelightAction implements Action {
        private final int pollRate;
        private boolean initialized = false;

        public StartLimelightAction(int pollRate) {
            this.pollRate = pollRate;
        }

        public StartLimelightAction() {
            this(100); // Default poll rate
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                startLimelight(pollRate);
                packet.put("limelight_started", true);
                packet.put("poll_rate", pollRate);
                initialized = true;
            }
            return false; // Action completes immediately
        }
    }

    /**
     * Action to close/stop the Limelight
     * Note: This Action is designed for single use. Create a new instance for each
     * use.
     */
    public class CloseLimelightAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                closeLimeLight();
                packet.put("limelight_closed", true);
                initialized = true;
            }
            return false; // Action completes immediately
        }
    }

    /**
     * Action to set the Limelight pipeline
     * Note: This Action is designed for single use. Create a new instance for each
     * use.
     */
    public class SetPipelineAction implements Action {
        private final Pipelines pipeline;
        private boolean initialized = false;

        public SetPipelineAction(Pipelines pipeline) {
            this.pipeline = pipeline;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                setPipeline(pipeline);
                packet.put("pipeline_set", pipeline.name());
                initialized = true;
            }
            return false; // Action completes immediately
        }
    }

    /**
     * Action to detect and retrieve an AprilTag
     * Note: Assumes Limelight is already started and APRILTAGGER pipeline is
     * active.
     * This Action is designed for single use. Create a new instance for each use.
     */
    public class GetAprilTagAction implements Action {
        private int attempts = 0;
        private static final int MAX_ATTEMPTS = 30; // Try for up to 30 polling cycles
        private LLResultTypes.FiducialResult result = null;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            attempts++;

            // Just read the latest result without changing Limelight state
            LLResult llResult = getLatestResult();
            if (llResult != null && llResult.isValid()) {
                java.util.List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    result = fiducials.get(0);
                    packet.put("apriltag_found", true);
                    packet.put("apriltag_id", result.getFiducialId());
                    packet.put("apriltag_family", result.getFamily());
                    packet.put("apriltag_x", result.getTargetXDegrees());
                    packet.put("apriltag_y", result.getTargetYDegrees());
                    return false; // Found! Action completes
                } else {
                    packet.put("apriltag_found", false);
                    packet.put("apriltag_attempt", attempts);
                }
            } else {
                packet.put("apriltag_found", false);
                packet.put("apriltag_result", "invalid");
                packet.put("apriltag_attempt", attempts);
            }

            // Continue trying until max attempts
            if (attempts >= MAX_ATTEMPTS) {
                packet.put("apriltag_status", "timeout");
                return false; // Give up after max attempts
            }

            return true; // Keep trying
        }

        public LLResultTypes.FiducialResult getResult() {
            return result;
        }
    }

    /**
     * Action to detect artifact sequence (motif) from AprilTags and store it for
     * match duration.
     * Designed to be called once at the beginning of auto.
     * Note: Assumes Limelight is already started and APRILTAGGER pipeline is
     * active.
     * This Action is designed for single use. Create a new instance for each use.
     */
    public class DetectArtifactSequenceAction implements Action {
        private int attempts = 0;
        private static final int MAX_ATTEMPTS = 30; // Try for up to 30 polling cycles
        private Motif detectedMotif = null;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            attempts++;

            // Read AprilTags without changing Limelight state
            LLResult result = getLatestResult();
            Motif found = null;

            if (result != null && result.isValid()) {
                java.util.List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    // Iterate and pick the first fiducial that maps to a known Motif
                    for (LLResultTypes.FiducialResult fr : fiducials) {
                        int id = fr.getFiducialId();
                        Motif m = motifFromId(id);
                        if (m != null) {
                            found = m;
                            packet.put("apriltag_id", id);
                            break;
                        }
                    }
                    if (found == null) {
                        packet.put("apriltag", "no matching motif");
                        packet.put("apriltag_attempt", attempts);
                    }
                } else {
                    packet.put("apriltag", "none");
                    packet.put("apriltag_attempt", attempts);
                }
            } else {
                packet.put("apriltag_result", "invalid");
                packet.put("apriltag_attempt", attempts);
            }

            // If we found a motif, store it and complete
            if (found != null) {
                detectedMotif = found;
                storedMotif = found;
                packet.put("motif_detected", found.name());
                packet.put("motif_stored", true);
                return false; // Found! Action completes
            }

            // Continue trying until max attempts
            if (attempts >= MAX_ATTEMPTS) {
                packet.put("motif_detected", "none");
                packet.put("motif_stored", false);
                packet.put("motif_status", "timeout");
                return false; // Give up after max attempts
            }

            return true; // Keep trying
        }

        public Motif getDetectedMotif() {
            return detectedMotif;
        }

        // Helper to map AprilTag ID to Motif enum. Returns null if no mapping exists.
        private Motif motifFromId(int id) {
            for (Motif m : Motif.values()) {
                if (m.getValue() == id)
                    return m;
            }
            return null;
        }
    }

    // Convenience methods to create actions

    /**
     * Create an action to start the Limelight
     * 
     * @param pollRate polling rate in Hz
     * @return StartLimelightAction
     */
    public Action startLimelightAction(int pollRate) {
        return new StartLimelightAction(pollRate);
    }

    /**
     * Create an action to start the Limelight with default polling rate
     * 
     * @return StartLimelightAction
     */
    public Action startLimelightAction() {
        return new StartLimelightAction();
    }

    /**
     * Create an action to close the Limelight
     * 
     * @return CloseLimelightAction
     */
    public Action closeLimelightAction() {
        return new CloseLimelightAction();
    }

    /**
     * Create an action to set the pipeline
     * 
     * @param pipeline the pipeline to set
     * @return SetPipelineAction
     */
    public Action setPipelineAction(Pipelines pipeline) {
        return new SetPipelineAction(pipeline);
    }

    /**
     * Create an action to get an AprilTag
     * 
     * @return GetAprilTagAction
     */
    public Action getAprilTagAction() {
        return new GetAprilTagAction();
    }

    /**
     * Create an action to detect artifact sequence
     * 
     * @return DetectArtifactSequenceAction
     */
    public Action detectArtifactSequenceAction() {
        return new DetectArtifactSequenceAction();
    }

}
