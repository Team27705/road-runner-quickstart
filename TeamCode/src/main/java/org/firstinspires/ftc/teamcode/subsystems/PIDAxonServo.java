package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDAxonServo {
    private Servo servo;
    private AnalogInput encoder;

    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private double tolerance = 0.02; // normalized position units
    private double maxIntegral = 0.5;
    private ElapsedTime timer = new ElapsedTime();

    private double targetPosition = 0;
    private boolean holdingPosition = false;

    // Axon encoder typically outputs 0-3.3V over its range
    private double voltageMin = 0.0;
    private double voltageMax = 3.3;
    private double positionMin = 0.0;
    private double positionMax = 1.0;

    public PIDAxonServo(HardwareMap hardwareMap, String servoName, String encoderName, double[] coeffs) {
        this.servo = hardwareMap.get(Servo.class, servoName);
        this.encoder = hardwareMap.get(AnalogInput.class, encoderName);
        this.kP = coeffs[0];
        this.kI = coeffs[1];
        this.kD = coeffs[2];
    }

    public void setTargetPosition(double target) {
        this.targetPosition = Range.clip(target, positionMin, positionMax);
        this.holdingPosition = true;
        resetPID();
    }

    public void update() {
        if (!holdingPosition) return;

        double currentPosition = getCurrentPosition();
        double error = targetPosition - currentPosition;
        double deltaTime = timer.seconds();
        timer.reset();

        if (deltaTime == 0) return;

        double proportional = kP * error;

        integralSum += error * deltaTime;
        integralSum = Range.clip(integralSum, -maxIntegral / kI, maxIntegral / kI);
        double integral = (kI != 0) ? kI * integralSum : 0;

        double derivative = kD * (error - lastError) / deltaTime;
        lastError = error;

        double output = proportional + integral + derivative;

        // Convert PID output to servo position adjustment
        double newPosition = currentPosition + output;
        servo.setPosition(Range.clip(newPosition, 0, 1));
    }

    public double getCurrentPosition() {
        double voltage = encoder.getVoltage();
        return Range.scale(voltage, voltageMin, voltageMax, positionMin, positionMax);
    }

    public double getCurrentAngle() {
        // Assumes 270-degree Axon range, adjust if using different model
        return getCurrentPosition() * 355;
    }

    public void setTargetAngle(double degrees) {
        setTargetPosition(degrees / 355);
    }

    public boolean atTarget() {
        return Math.abs(targetPosition - getCurrentPosition()) <= tolerance;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getError() {
        return targetPosition - getCurrentPosition();
    }

    public void setDirectPosition(double position) {
        holdingPosition = false;
        servo.setPosition(Range.clip(position, 0, 1));
    }

    public void setPIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public void setEncoderRange(double voltageMin, double voltageMax) {
        this.voltageMin = voltageMin;
        this.voltageMax = voltageMax;
    }

    public void setPositionRange(double min, double max) {
        this.positionMin = min;
        this.positionMax = max;
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    public double getVoltage() {
        return encoder.getVoltage();
    }

    private void resetPID() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public void stop() {
        holdingPosition = false;
    }


    @TeleOp(name = "Torctex max vel", group = "testing")
    public static class TRServo extends LinearOpMode {


        private double kP = 0;
        private double kI = 0;
        private double kD = 0;
        private double[] coeffs = {kP, kI, kD};

        @Override
        public void runOpMode () throws InterruptedException{
            PIDAxonServo servo = new PIDAxonServo(this.hardwareMap, "", "", coeffs);


        }

    }
}
