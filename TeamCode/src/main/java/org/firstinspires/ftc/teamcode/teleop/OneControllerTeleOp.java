package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "One Controller TeleOp")
public class OneControllerTeleOp extends LinearOpMode {

    private MecanumDrive driveTrain;
    private Intake intake;
    private NewOuttake outtake;
    private Spindexer spindexer;
    private Pose2d initialPosition;

    private List<Action> runningActions = new ArrayList<>();
    private ElapsedTime runTime;

    @Override
    public void runOpMode() throws InterruptedException {
        initialPosition = new Pose2d(0, 0, 0);

        driveTrain = new MecanumDrive(this.hardwareMap, initialPosition);
        intake = new Intake(this.hardwareMap);
        outtake = new NewOuttake(this.hardwareMap);
        // Spindexer now listens to gamepad1
        spindexer = new Spindexer(this.hardwareMap, false);

        runTime = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {
            // All updates now pass gamepad1
            driveLogic();
            intakeLogic();
            outtakeLogic();
            spindexer.update(gamepad1);

            outtake.updatePID();

            // Roadrunner Action System (Placeholders for TeleOp actions)
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                if (action.run(null)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            updateTelem();
        }
    }

    private void driveLogic() {
        double leftX = gamepad1.left_stick_x * 1.1;
        double leftY = -gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x;

        // Deadzones
        if (Math.abs(leftY) < 0.1) leftY = 0;
        if (Math.abs(leftX) < 0.1) leftX = 0;
        if (Math.abs(rightX) < 0.1) rightX = 0;

        driveTrain.driverRelativePower(leftY, leftX, rightX);
    }

    private void intakeLogic() {
        if (gamepad1.left_bumper) {
            intake.spinIntake();
        } else if (gamepad1.right_bumper) {
            intake.reverseIntake();
        } else if (gamepad1.b) {
            intake.idleIntake();
        }
    }

    private void outtakeLogic() {
        // Presets mapped to D-pad
        if (gamepad1.dpad_up) { // Close shot
            outtake.setHoodAngle(0.25);
            outtake.setTargetVel(780);
        } else if (gamepad1.dpad_down) { // Far shot
            outtake.setHoodAngle(0.7);
            outtake.setTargetVel(1300);
        } else if (gamepad1.dpad_right) { // Mid shot
            outtake.setHoodAngle(0.5);
            outtake.setTargetVel(1000);
        } else if (gamepad1.dpad_left) { // Stop Shooter
            outtake.setTargetVel(0);
        }
    }

    private void updateTelem() {
        telemetry.addLine(outtake.outtakeLog());

        String[] motif = spindexer.getCurrentMotif();
        if (motif != null && motif.length >= 3) {
            telemetry.addData("Motif", "%s, %s, %s", motif[0], motif[1], motif[2]);
        }

        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();
    }
}