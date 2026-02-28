package org.firstinspires.ftc.teamcode.telop;



import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.ArrayList;
import java.util.List;


@TeleOp (name = "")
public class DriverTelop extends LinearOpMode {

    private MecanumDrive driveTrain;
    private Intake intake;
    private NewOuttake outtake;
    private Spindexer spindexer;
    private Pose2d intialPosition;

//    private Telemetry telemetry;

    private List<Action> runningActions = new ArrayList<>();

    private ElapsedTime runTime;


    @Override
    public void runOpMode() throws InterruptedException {

        intialPosition = new Pose2d(0,0,0);


        driveTrain = new MecanumDrive(this.hardwareMap, intialPosition);
        intake = new Intake(this.hardwareMap);
        outtake = new NewOuttake(this.hardwareMap);
        spindexer = new Spindexer(this.hardwareMap, false);


        runTime = new ElapsedTime();

        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (opModeIsActive()) {
            //create an action scheduler here and then add actions from detecting userInputs
            runTime.reset();
            controllerBehaviorA();
            controllerBehaviorB();
            outtake.updatePID();


//            List<Action> newActions = new ArrayList<>();
//            for (Action action: runningActions) {
//
//            }
//            runningActions = newActions;
            telemetry.addData("Loop Time: ", runTime.milliseconds());
            telemetry.update();

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

        if (gamepad2.dpadUpWasReleased()) { // close shot
            outtake.setHoodAngle(.7);
            outtake.setTargetVel(800);

        }
        else if (gamepad2.dpadDownWasReleased()) { //far shot
            outtake.setHoodAngle(.5);
            outtake.setTargetVel(950);
        }
        else if (gamepad2.dpadRightWasReleased()){ //mid shot
            outtake.setHoodAngle(.25);
            outtake.setTargetVel(800);
        }

        spindexer.update(gamepad2); //start and x are used
        telemetry.addLine(outtake.atTarget());
    }

    public void updateTelem () {
    }
}
