package org.firstinspires.ftc.teamcode.telop;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.Spindexer;

import java.util.ArrayList;
import java.util.List;


@TeleOp (name = "")
public class DriverTelop extends LinearOpMode {

    private MecanumDrive driveTrain;
    private Intake intake;
    private Outtake outtake;
    private Spindexer spindexer;
    private Pose2d intialPosition;

    private Telemetry telemetry;

    private List<Action> runningActions = new ArrayList<>();


    @Override
    public void runOpMode() throws InterruptedException {

        intialPosition = new Pose2d(0,0,0);


        driveTrain = new MecanumDrive(this.hardwareMap, intialPosition);
        intake = new Intake(this.hardwareMap);
        outtake = new Outtake(this.hardwareMap);
        spindexer = new Spindexer(this.hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            //create an action scheduler here and then add actions from detecting userInputs



            controllerBehaviorA();
            controllerBehaviorB();


//            List<Action> newActions = new ArrayList<>();
//            for (Action action: runningActions) {
//
//            }
//            runningActions = newActions;

            updateTelem();
        }



    }

    public void controllerBehaviorA () {
        double leftX = gamepad1.left_stick_x * 1.1;
        double leftY = -gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x ;

        if (leftY >= -0.1  && leftY <= 0.1) {
            leftY = 0;
        }
        if (leftX >= -0.1 && leftX <= 0.1) {
            leftX = 0;
        }
        if (rightX >= -0.1 && rightX <= 0.1) {
            rightX = 0;
        }

        driveTrain.driverRelativePower(leftY, leftX , rightX);


        if (gamepad1.left_bumper) {
            intake.spinIntake();
        }
        if (gamepad1.right_bumper) {
            intake.idleIntake();
        }


    }

    public void controllerBehaviorB () {
        if (gamepad2.x) {
            outtake.runOuttake();
        }

        if (gamepad2.b) {
            outtake.idle();
        }
        if (gamepad2.a) {
            outtake.stop();
        }

        if (gamepad2.dpad_up) {
            spindexer.kickServo();
        }
        if (gamepad2.dpad_down) {
            spindexer.resetServo();
        }

        if (gamepad2.y) {
            spindexer.cycleSpindexer();
        }

//        if (gamepad2.right_bumper) {
//            spindexer.reAdjust();
//        }
    }

    public void updateTelem () {
//        RevColorSensorV3 colorSensor = spindexer.getColorSensor();
//
//        telemetry.addData("Red:", colorSensor.red() );
//        telemetry.addData("Green:", colorSensor.green() );
//        telemetry.addData("Blue:", colorSensor.blue() );
//        telemetry.update();

    }
}
