package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class NewOuttake {

    private static double kP, kI, kD; //PID coeffs, set to private after tuning
    private final DcMotorEx flywheelMotorTop;
    private final DcMotorEx flywheelMotorBottom;
    private double currentTargetVelocity = 1500;
    private double currentVelocity;
    private boolean readyToShoot;
    //coefficents
    private final double kV = 0.000357142857;
    private double kS; //kv should be 1 / maxVelocity from encoder, create a short op mod for that
    private final Servo hoodServo;
    private final InterpLUT lut;


    //notes: 1 PIDFCoefficents for both motors, use the numbers for one for both
    //kS: smallest number to get the motor to spin, overcoming friction
    //kv is amount of power needed to get to max velocity / max velocity, so its 1/max velocity
    //that way if target velo is = max velo, then it will give you the max voltage or power that the motor can use
    //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
    //https://www.youtube.com/watch?v=lzVuUpwNdZo
    //https://docs.ftclib.org/ftclib/features/util


    //(kP * (targetVelocity - left.getVelocity()) + kV * targetVelocity) * 12 / volts; volts is optional, voltage sensor
    //


    public NewOuttake(HardwareMap hardwareMap) {
        flywheelMotorTop = hardwareMap.get(DcMotorEx.class, "Flywheel Top");
        flywheelMotorBottom = hardwareMap.get(DcMotorEx.class, "Flywheel Bottom");

//        flywheelMotorTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        flywheelMotorBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        flywheelMotorTop.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotorBottom.setDirection(DcMotorSimple.Direction.FORWARD);

        hoodServo = hardwareMap.get(Servo.class, "Hood Servo");
        hoodServo.setDirection(Servo.Direction.REVERSE);
        flywheelMotorTop.getVelocity();

        lut = new InterpLUT();
//        lut.add(); //1st distance, second target velocity
        lut.createLUT();

    }

    public void updatePID() {
        currentVelocity = flywheelMotorTop.getVelocity(); // max vel in ticks per second should be 2800

        double error = currentTargetVelocity - currentVelocity;
        double feedback = error * kP;
        double feedforward = kV;
        //prob want to implement some deadzone check for error and break out of updatePID while returning
        flywheelMotorTop.setPower(feedback + feedforward);
        flywheelMotorBottom.setPower(feedback + feedforward);
    }

    //https://docs.ftclib.org/ftclib/features/util#what-is-a-look-up-table
    //no need to use this in auto, just do 1 pos, only use for teleop
    //check if shooting mode active in teleop loop then run limelight, autoUpdateTargetVel and feed limelight pos
    //get vectorDistance from limelight
    public void autoUpdateTargetVel(Pose2d vectorDistance) {
        double distance = Math.sqrt(Math.pow(vectorDistance.component1().x, 2)
                + Math.pow(vectorDistance.component1().y, 2));
        //prob check if its 0,0,0
        currentTargetVelocity = lut.get(distance);
    }

    //TODO: Implement small InterpLUT for hood
    public void autoUpdateHood(Pose2d vectorDistance) {
        double distance = Math.sqrt(Math.pow(vectorDistance.component1().x, 2)
                + Math.pow(vectorDistance.component1().y, 2));
        if (distance <= 10) { //def incorrect fix later, only should have two or angles tho to reduce time to test and variability
            setHoodAngle(.1);
        }
        if (distance > 40) {
            setHoodAngle(.5);
        }
    }

    public double getHoodAngle() {
        return hoodServo.getPosition();
    }

    public void setHoodAngle(double angle) {
        hoodServo.setPosition(angle);
    }

    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format(
                "currentTargetVelocity: " +
                        "Current Velocity: " +
                        "Current Hood Angle: ",
                currentTargetVelocity,
                currentVelocity,
                hoodServo.getPosition()


        );
    }

    //https://youtu.be/aPNCpZzCTKg?t=786
    @TeleOp(name = "FlyWheel Tuner", group = "test")
    public static class FlyWheelTuner extends OpMode {
        public static double targetVelocity, velocity;
        public static double P, V, S; //do not need kI or kD , find starting numbers
        private DcMotorEx flywheelMotorTop, flywheelMotorBottom;

        @Override


        // TODO: ETHAN READ THIS -> Here's how you tune it IN THE CORRECT ORDER
        //
        //kS stands for "static." It's the greatest power you can give the flywheel motor(s) without it moving.
        //So it should be a really small number below like 0.05 or something.
        //You should increase it until it begins to move and then decrease it until it stops moving again.
        //
        // kV is the next number you tune. Pick a velocity like 1500 or something and then increase kV until it gets there.
        //Tune P. Send a ball through the launcher and increase P until it doesn't overshoot past the target velocity in the recovery phase.
        // Keep increasing though because unless you see overshoot you're fine.
        //Tune I. After all the other numbers are tuned, save them, and then make kS slightly negative.
        // This will simulate a drop in power to the motor from a low battery. Increase I until your confident that you can still get to your target velocity.
        //DO ALL OF THIS ON FTCDashboard
        //

        public void init() {
            flywheelMotorTop = hardwareMap.get(DcMotorEx.class, "Flywheel Top");
            flywheelMotorBottom = hardwareMap.get(DcMotorEx.class, "Flywheel Bottom");
            flywheelMotorTop.setDirection(DcMotorSimple.Direction.FORWARD);
            flywheelMotorBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        @Override
        public void loop() {
            velocity = flywheelMotorTop.getVelocity();
            telemetry.addData("TargetVel", targetVelocity);
            telemetry.addData("CurrentVel", velocity);
            double error = targetVelocity - velocity;
            double feedback = error * P;
            double feedforward = V * targetVelocity + S;
            flywheelMotorTop.setPower(feedback + feedforward);
            flywheelMotorBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    @TeleOp(name = "Flywheel distance tests", group = "Test")
    public static class DistanceTests extends LinearOpMode {
        //Only change targetVelocity and hood angle aka servo angle
        //ideally only want to change 1 number
        //do it based off of straight line position in inches from limelight, may want to combine with limelight calls


        //**
        // Ya, if you add kV * targetVelocity to your PID power it helps a lot. You can also add a constant power kS to account for static friction,
        // but a lot of teams find it unnecessary. Also, don't use D, just P and I. If you want to compensate for battery loss,
        // I'd recommend multiplying your power by (tuned voltage / actual voltage) same as what he said basically, just a bit better. Here are steps for tuning...
        //
        //

        @Override
        public void runOpMode() {
            waitForStart();

            while (!isStopRequested()) {

            }
        }
    }
}
