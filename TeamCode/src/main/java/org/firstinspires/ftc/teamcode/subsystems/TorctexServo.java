package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


public class TorctexServo {

    private final CRServo trServo;
    private final AnalogInput servoEncoder;
    private double currentAngle;
    private double previousAngle;
    private double startingAngle; //last angle
    private double targetAngle;
    private double totalRotation; //the unnormalized vector, the total number of rotations

    private double homeAngle;

    private double angleDelta;

    private int wraps; //tracks the number of times the encoder has wrapped around

    private boolean  initalized = false;

    //the positions


    private DIRECTION direction;

    public enum DIRECTION {
        REVERSE,
        FORWARD
    }


    public TorctexServo (String servoName, String encoderName, Servo.Direction direction) {
        trServo = hardwareMap.get(CRServo.class, servoName);
        servoEncoder = hardwareMap.get(AnalogInput.class, encoderName);

    }

    public TorctexServo (CRServo servo, AnalogInput analogInput) {
        trServo = servo;
        servoEncoder = analogInput;

        wraps = 0;

        direction = DIRECTION.FORWARD;
    }

    public void initialize() {
        trServo.setPower(0);
        previousAngle = getCurrentAngle();
        currentAngle = startingAngle;
        initalized = true;
    }

//    public boolean atPosition () {
//        return currentAngle - <  targetAngle;
//    }

    public synchronized void update() {
        // run this contionously
        //calculate unnormalized rotations in degrees
        //calculate total voltage instead then convert to angles then mod to by 360
        //create a function to accumalte and invert the difference in voltage when direction is REVERESE
        //cliffs from like 3.255 and 0.005
        //0.55 - 3.255 ? like 4 degrees of inaccuracy 2 and 2 each side

        if (!initalized) return;


        currentAngle = getCurrentAngle();
        angleDelta = currentAngle - previousAngle;

        if (angleDelta > 300) { //this wont work when direction is negative
            wraps++;
        }
        else if (angleDelta < -300) {
            wraps--;
        }

//        if (angleDelta > 180) {
//            totalRotation -= angleDelta - 360;
//        }
//        else if (angleDelta < -180) {
//            totalRotation += angleDelta + 360;
//        }

//        angleDelta = totalRotation - previousAngle;

        totalRotation = angleDelta + (wraps * 360);

        previousAngle = currentAngle;


        //include an interupt if the servo is already at the

//        position += currentAngle - previousAngle; // calculate the total rotations
//        if (currentAngle != targetAngle) {
//            setPower();
//        }
    }


//    public void updateTotalVoltage () {
//
//
//        if () {
//
//        }
//        if () {
//
//        }
//    }

    public double getVoltage () {
        //read the analog voltage of the Servo
        return servoEncoder.getVoltage();
    }
    public double getMaxVoltage () {
        return servoEncoder.getMaxVoltage();
    }

    public double getCurrentAngle() {
        //takes the Analog Voltage Signal from the AnalogInput of the Servo to calculate the angle by normalizing it (divide by maximum voltage)
        double angle = (int) ((getVoltage() / 3.3) * 360);
        if (direction.equals(DIRECTION.REVERSE)) {
            angle = -1 * angle;
        }

        return angle;
    }

    public void setServoPower (double power) {
        trServo.setPower(power);
    }

    public void changeTargetPosition () {
    }

    public void setTargetPosition() {

    }

    public double getTotalRotation() {
        return totalRotation;
    }

    public double getAngleDelta () {
        return angleDelta;
    }


    @SuppressLint("DefaultLocale")
    public String log() {

        return String.format(
                "Starting Position: %.3f\n" +
                        "Voltage Reading: %.3f\n"+
                        "Current Angle: %.3f\n"+
                        "Total angular Rotation: %.3f \n"+
                        "Angle Change: %.3f \n",
//                        "Target Position: %.3\n"+
//                        "Error: %.3\n",

                    startingAngle,
                    getVoltage(),
                    getCurrentAngle(),
                    getTotalRotation(),
                    getAngleDelta()
                );
    }


    @TeleOp(name = "Torctex Servo Analog Input test", group = "test")
    public static class TorctexServoTest extends LinearOpMode {
        @Override

        public void runOpMode () throws InterruptedException {

            waitForStart();
            CRServo sr = hardwareMap.get(CRServo.class, "rightHorizSlide");
            AnalogInput analogVoltageSignal = hardwareMap.get(AnalogInput.class, "rightHorizSlideEncoder");

            TorctexServo TRServo = new TorctexServo(sr, analogVoltageSignal);
            ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            double timeElapsed = 0;
            double start = 0;

            double now = 0;
            double last = 0;

            while (!isStopRequested()) {
                TRServo.update();
                if (gamepad1.aWasReleased()) {
                    clock.reset();
                    start = clock.now(TimeUnit.MILLISECONDS);
                    TRServo.initialize();
                    TRServo.setServoPower(.2);
                }


                TRServo.update();
                now = clock.now(TimeUnit.MILLISECONDS);
                timeElapsed = now - last;
                telemetry.addLine(TRServo.log());
                telemetry.addData("Time ms:", "%.3f", timeElapsed);
                telemetry.addData("Velocity:", "%.3f", TRServo.getAngleDelta() / timeElapsed);
                telemetry.update();
                last = now;

            }

            //doing clock / time inside of


        }
    }

}
