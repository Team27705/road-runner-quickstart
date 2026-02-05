package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class TorctexServo {

    private final CRServo TRServo;
    private final AnalogInput servoEncoder;

    private int currentAngle;

    private int startingAngle; //last angle

//    private double voltage

//    private final

    private DIRECTION direction;

    public enum DIRECTION {
        REVERSE,
        FORWARD
    }





    public TorctexServo (String servoName, String encoderName, Servo.Direction direction) {
        TRServo = hardwareMap.get(CRServo.class, servoName);
        servoEncoder = hardwareMap.get(AnalogInput.class, encoderName);

    }

    public TorctexServo (CRServo servo, AnalogInput analogInput) {
        TRServo = servo;
        servoEncoder = analogInput;

        direction = DIRECTION.FORWARD;
    }

    public double getVoltage () {
        //read the analog voltage of the Servo
        return servoEncoder.getVoltage();
    }

    public double getMaxVoltage () {
        return servoEncoder.getMaxVoltage();
    }

    public void changeTargetPosition () {

    }

    public void setTargetPosition() {

    }


    public int updateCurrentAngle() {
        int angle = (int) ((getVoltage() / 3.3) * 360);
        if (direction.equals(DIRECTION.REVERSE)) {
            angle = -1 * angle;
        }

        return angle;
    }
    @TeleOp(name = "Torctex Servo Analog In test", group = "test")
    public static class TorctexServoTest extends LinearOpMode {
        @Override

        public void runOpMode () throws InterruptedException{

            waitForStart();
            CRServo sr = hardwareMap.get(CRServo.class, "rightHorizSlide");
            AnalogInput analogVoltageSignal = hardwareMap.get(AnalogInput.class, "rightHorizSlideEncoder");

            TorctexServo Servo = new TorctexServo(sr, analogVoltageSignal);


            while (!isStopRequested()) {
                telemetry.addData("Voltage Reading: %.6f", Servo.getVoltage());

                telemetry.addData("Max Voltage Reading: %.3f", Servo.getMaxVoltage());

                telemetry.update();
            }


        }
    }

}
