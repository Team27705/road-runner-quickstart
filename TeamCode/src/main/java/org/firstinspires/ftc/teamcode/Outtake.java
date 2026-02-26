package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
public class Outtake {

    //https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit?pli=1&tab=t.0


    private final DcMotor flywheelMotor1; //DcMotorEx is for advanced behavior like PIDS
    private final DcMotor flywheelMotor2;
    private PIDCoefficients flywheelCoeffs;
    public Outtake (HardwareMap hardwareMap) {
        PIDCoefficients flywheelCoeffs = new PIDCoefficients();
        flywheelMotor1 = hardwareMap.get(DcMotor.class, "flywheelMotor1");
        flywheelMotor2 = hardwareMap.get(DcMotor.class, "flywheelMotor2");
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor1.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotor.Direction.REVERSE);
    }

    public void runOuttakeFar() {
        flywheelMotor1.setPower(0.80);
        flywheelMotor2.setPower(0.80);
    }

    public void runOuttakeClose() {
        flywheelMotor1.setPower(0.65);
        flywheelMotor2.setPower(0.65);
    }

    public void idle() {
        flywheelMotor1.setPower(0.35);
        flywheelMotor2.setPower(0.35);
    }
    public void stop () { flywheelMotor1.setPower(0);
    flywheelMotor2.setPower(0);}

    public void ramp () {
        flywheelMotor1.setPower(1);
        flywheelMotor2.setPower(1);
    }

    public class AutoRamp implements Action {

        private boolean initalized = false;
        private double motorPower = 0.0;

        public AutoRamp(double motorPower) {
            this.motorPower = motorPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            flywheelMotor1.setPower(motorPower);
            flywheelMotor2.setPower(motorPower);
            return true;
        }
    }

    public Action AutoRamp (double motorPower) {
        return new AutoRamp(motorPower); // constructs a AutoRamp Action and returns it
    }

}

