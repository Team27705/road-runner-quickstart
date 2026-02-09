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
    private double position; //the unnormalized vector, the total number of rotations

    private double homeAngle;

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

        direction = DIRECTION.FORWARD;


        initialize();

    }

    public void initialize() {
        trServo.setPower(0);
        previousAngle = getCurrentAngle();

        currentAngle = startingAngle;
    }

//    public boolean atPosition () {
//        return currentAngle - <  targetAngle;
//    }

    public void update() {
        // run this contionously
        //calculate unnormalized rotations in degrees
//
//
//
        currentAngle = getCurrentAngle();

        position += currentAngle - previousAngle; // calculate the total rotations
//        if (currentAngle != targetAngle) {
//            setPower();
//        }
        previousAngle  = getCurrentAngle();
    }

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

    public double getPosition() {
        return position;
    }


    @SuppressLint("DefaultLocale")
    public String log() {

        return String.format(
                "Starting Position: %.3f\n" +
                        "Voltage Reading: %.3f\n"+
                        "Current Angle: %.3f\n",
                        "Total angular Rotation: %.3f \n",
//                        "Target Position: %.3\n"+
//                        "Error: %.3\n",

                    startingAngle,
                    getVoltage(),
                    getCurrentAngle(),
                    getPosition()
                );
    }


    @TeleOp(name = "Torctex Servo Analog In test", group = "test")
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


            while (!isStopRequested()) {
                if (gamepad1.aWasReleased()) {
                    start = clock.startTime();
                }
                TRServo.setServoPower(1);


                telemetry.addLine(TRServo.log());
                telemetry.addData("Time ms:", timeElapsed);
                telemetry.update();


                double end = clock.time(TimeUnit.MILLISECONDS);
                timeElapsed = start - end;
            }


        }
    }

}
