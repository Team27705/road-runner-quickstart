package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.RTPTorctex;

public class NewSpindexer {
    //include spindexer servo, light, booktkicker, and color sensor
    //note:deprecate Spindexer.java once done
    private RTPTorctex spindexer;
    private CRServo crServo;
    private AnalogInput analogEncoder;

    private Servo bootkicker;
    private RevColorSensorV3 colorSensor;
    private String[] storage = {"","",""};


    public NewSpindexer (HardwareMap hardwareMap) {
        //Spindexer Servo
        crServo = hardwareMap.get(CRServo.class, "Spindexer Servo");
        analogEncoder = hardwareMap.get(AnalogInput.class, "Spindexer Encoder");
        spindexer = new RTPTorctex(crServo, analogEncoder);


        bootkicker = hardwareMap.get(Servo.class, "bootkicker");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

    }

    public void update () {
        //update spindexer PID
        spindexer.update();

        if (!detectGreen() || !detectPurple) return; //no colors detected do nothing

        if (bootkicker.)
        //if color is detected, then set targetPosition of Spindexer

    }

    public boolean detectGreen () {

    }

    public boolean detectPurple () {

    }



}
