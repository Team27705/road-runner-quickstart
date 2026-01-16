package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
public class Outtake {

    //https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit?pli=1&tab=t.0


    private final DcMotor flywheelMotor; //DcMotorEx is for advanced behavior like PIDS
    private PIDCoefficients flywheelCoeffs;
    public Outtake (HardwareMap hardwareMap) {
        PIDCoefficients flywheelCoeffs = new PIDCoefficients();
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheelMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runOuttake() {
        flywheelMotor.setPower(0.80);
    }

    public void idle() {
        flywheelMotor.setPower(0.35);
    }
    public void stop () { flywheelMotor.setPower(0);}

    public void ramp () {
        flywheelMotor.setPower(1);
    }

    public class AutoRamp implements Action {

        private boolean initalized = false;
        private double motorPower = 0.0;

        public AutoRamp(double motorPower) {
            this.motorPower = motorPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            flywheelMotor.setPower(motorPower);
            return true;
        }
    }

    public Action AutoRamp (double motorPower) {
        return new AutoRamp(motorPower); // constructs a AutoRamp Action and returns it
    }

}

