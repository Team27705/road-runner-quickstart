package org.firstinspires.ftc.teamcode.telop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "RC Teleop")
public class RCTeleop extends LinearOpMode {

    private MecanumDrive driveTrain;
    private Intake intake;
    private NewOuttake outtake;
    private Spindexer spindexer;
    private VisionPortal visionPortal;

    private List<Action> runningActions = new ArrayList<>();
    private ElapsedTime runTime;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- SUBSYSTEM INIT ---
        driveTrain = new MecanumDrive(this.hardwareMap, new Pose2d(0, 0, 0));
        intake = new Intake(this.hardwareMap);
        outtake = new NewOuttake(this.hardwareMap);
        spindexer = new Spindexer(this.hardwareMap, false);
        runTime = new ElapsedTime();

        // --- WEBCAM INIT ---
        // This sets up the stream so you can view it on the Driver Hub
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .build();

        telemetry.addLine("Wait for camera to initialize...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driveLogic();
            intakeLogic();
            outtakeLogic();
            spindexer.update(gamepad1);
            outtake.updatePID();

            // RR Action Loop
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                if (action.run(null)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            updateTelem();
        }

        // Clean up camera on stop
        visionPortal.close();
    }

    private void driveLogic() {
        double leftX = gamepad1.left_stick_x * 1.1;
        double leftY = -gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x;

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
        if (gamepad1.dpad_up) {
            outtake.setHoodAngle(0.25);
            outtake.setTargetVel(780);
        } else if (gamepad1.dpad_down) {
            outtake.setHoodAngle(0.7);
            outtake.setTargetVel(1300);
        } else if (gamepad1.dpad_right) {
            outtake.setHoodAngle(0.5);
            outtake.setTargetVel(1000);
        } else if (gamepad1.dpad_left) {
            outtake.setTargetVel(0);
        }
    }

    private void updateTelem() {
        telemetry.addData("Camera Status", visionPortal.getCameraState());
        telemetry.addLine(outtake.outtakeLog());

        String[] motif = spindexer.getCurrentMotif();
        if (motif != null && motif.length >= 3) {
            telemetry.addData("Motif", "%s, %s, %s", motif[0], motif[1], motif[2]);
        }
        telemetry.update();
    }
}