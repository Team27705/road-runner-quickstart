package org.firstinspires.ftc.teamcode.limelight;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Limelight {
    // https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming

    private final Limelight3A limelight;
    private final HardwareMap hardwareMap;

    public Limelight(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null!");
        }
        this.hardwareMap = hardwareMap;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(Pipelines.APRILTAGGER.getValue());
    }

    @NonNull
    private static Pose2d getPose2d(LLResult result, double robotYaw) throws MT2CalculationFailedException, MT2PositionIsZeroedException {
        Pose3D botpose_mt2 = result.getBotpose_MT2();
        if (botpose_mt2 == null) {
            throw new MT2CalculationFailedException("Limelight failed to calculate botpose_mt2!");
        }
        double rawX = botpose_mt2.getPosition().x;
        double rawY = botpose_mt2.getPosition().y;

        if (rawX == 0.0 && rawY == 0.0) {
            throw new MT2PositionIsZeroedException(
                    "Limelight botpose_mt2 position is zeroed, the limelight wasn't confident in its calculation and returned 0.0 for both X and Y."
            );
        }

        // convert Limelight Pose3D (meters) to Roadrunner Pose2d (inches)
        final double METERS_TO_INCHES = 39.3700787;
        double x_in = rawX * METERS_TO_INCHES;
        double y_in = rawY * METERS_TO_INCHES;

        // finalize calculation, return pose
        return new Pose2d(new Vector2d(x_in, y_in), robotYaw);
    }

    public void switchPipeline(Pipelines pipeline) throws IllegalArgumentException {
        if (pipeline == null) {
            throw new IllegalArgumentException("Pipeline cannot be null!");
        }
        if (pipeline.getValue() < 0 || pipeline.getValue() > 2) {
            throw new IllegalArgumentException("Invalid pipeline value: " + pipeline.getValue());
        }
        limelight.pipelineSwitch(pipeline.getValue());
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public Pose2d getRobotPoseFromAprilTag() throws NoAprilTagDetectedException, MT2CalculationFailedException, MT2PositionIsZeroedException {
        limelight.start();
        switchPipeline(Pipelines.APRILTAGGER);

        try {
            MecanumDrive drive = hardwareMap.get(MecanumDrive.class, "drive");
            double robotYaw = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // give limelight the yaw so it can do its thing
            limelight.updateRobotOrientation(Math.toDegrees(robotYaw));

            // get data after orientation update
            LLResult result = getLatestResult();

            if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
                throw new NoAprilTagDetectedException("No AprilTags detected!");
            }

            return getPose2d(result, robotYaw);
        } finally {
            // Always stop to avoid keeping the camera active after a query.
            limelight.stop();
        }
    }

    public enum Pipelines {
        APRILTAGGER(0), PURPLER(1), GREENER(2);

        private final int value;

        Pipelines(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public static class NoAprilTagDetectedException extends Exception {
        public NoAprilTagDetectedException(String message) {
            super(message);
        }
    }

    public static class MT2CalculationFailedException extends Exception {
        public MT2CalculationFailedException(String message) {
            super(message);
        }
    }

    public static class MT2PositionIsZeroedException extends Exception {
        public MT2PositionIsZeroedException(String message) {
            super(message);
        }
    }
}