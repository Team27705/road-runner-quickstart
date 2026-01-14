package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class Spindexer {
    private Servo bootkicker;
    private DcMotor spindexerMotor;
    private RevColorSensorV3 colorSensor;

    private String[] artifactSequence; // set this with camera
    private String[] storageSequence = {"Empty","Empty","Empty"}; //let 1 be your intake, write this to dashboard for operator

    private ArrayList<String> artifactsInGoal; //have a button to add a green or purple and remove all elements from list
    //use the last item in the list to compare with the artifact sequence and
    //write to dashboard if needed

    private final double oneCycle = 1453.2 * 120 / 72;
    private final int error = 50;





    public Spindexer (HardwareMap hardwareMap) {
        bootkicker = hardwareMap.get(Servo.class, "bootkicker");
        spindexerMotor = hardwareMap.get(DcMotor.class, "spindexer");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        bootkicker.setDirection(Servo.Direction.REVERSE);

        spindexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void resetServo () {
        bootkicker.setPosition(0);
    }
    public void kickServo () {
        bootkicker.setPosition(.45);
    }
    public boolean servoStatus () {
        return bootkicker.getPosition() == 0.0;

    }


    public void cycleSpindexer () {
        spindexerMotor.setTargetPosition((int) oneCycle - error);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(.7);


    }

    public boolean spindexerAvailable () {
        //checks if the spindexer's encoder has been reset and if the servo
        if (spindexerMotor.getMode().equals(DcMotor.RunMode.STOP_AND_RESET_ENCODER) && servoStatus()) {
            return true;
        }
        else {
            return false;
        }
    }

    public void checkSpinStatus () { //checks if the spindexer has reached target position
        if (spindexerMotor.getCurrentPosition() == (int) oneCycle - error) { // maybe subtract a little from onecycle to account for momentum, also use != maybe?
            spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void readIntake () {
        //use color sensor
    }

    public void setArtifactSequence () {

    }

    public String colorDetected () {
        String rgb = "";

        rgb += "red:" + colorSensor.red();
        rgb += "\ngreen:" + colorSensor.green();
        rgb += "\nblue:" + colorSensor.blue();
        return rgb;
    }


    public final class SetIndexAuto implements Action {
        //makes the assumption that the spindexer has been preloaded with 3 balls
        private boolean initalized = false;
        public SetIndexAuto() {

        }


        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            if (!initalized) {
                for (int i = 0; i < 3; i++) {
                    //check color of ball for purple or green then add it to list, then turn
//                    if (colorSensor.blue() && colorSensor.red()) {
//
//                    }
                }
                initalized = true; // do this bc its a one time action
            }
            return true;
        }

    }

    public final class loadArtifact implements Action {
        // goes to next avaliable artifact that best matches the pattern by comparing
        //add button to
        public loadArtifact() {

        }


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            for (int i = 0; i < 3; i ++) {
                if (storageSequence[i].equals("")) {

                }
            }
            return true;
        }

    }

}
