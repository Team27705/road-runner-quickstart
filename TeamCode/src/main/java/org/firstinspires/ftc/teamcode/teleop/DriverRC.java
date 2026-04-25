package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp (name = "Driver RC", group = "Control")
public class DriverRC extends LinearOpMode {

    private MecanumDrive driveTrain;
    private Intake intake;
    private NewOuttake outtake;
    private Spindexer spindexer;
    private Pose2d intialPosition;

    // Vision Portal for the webcam
    private VisionPortal visionPortal;

    private List<Action> runningActions = new ArrayList<>();
    private ElapsedTime runTime;

    @Override
    public void runOpMode() throws InterruptedException {

        intialPosition = new Pose2d(0,0,0);

        driveTrain = new MecanumDrive(this.hardwareMap, intialPosition);
        intake = new Intake(this.hardwareMap);
        outtake = new NewOuttake(this.hardwareMap);
        spindexer = new Spindexer(this.hardwareMap, false);

        // --- WEBCAM INITIALIZATION ---
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .build();

        // Send the feed to FTC Dashboard (192.168.43.1:8080/dash)
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        runTime = new ElapsedTime();

        telemetry.addLine("Camera Initialized and Streaming to Dashboard");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            spindexer.update(gamepad2);
            controllerBehaviorA();
            outtake.updatePID();

            controllerBehaviorB();

            List<Action> newActions = new ArrayList<>();
            for (Action action: runningActions) {
                if (action.run(null)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            telemetry.update();
        }

        // Close camera on stop
        visionPortal.close();
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

        if (gamepad1.leftBumperWasReleased()) {
            intake.spinIntake();
        }
        else if (gamepad1.rightBumperWasReleased()) {
            intake.reverseIntake();
        }
        if (gamepad1.bWasReleased()) {
            intake.idleIntake();
        }
    }

    public void controllerBehaviorB () {
        if (gamepad2.dpadUpWasReleased()) { // close shot
            outtake.setHoodAngle(.25);
            outtake.setTargetVel(780);
        }
        else if (gamepad2.dpadDownWasReleased()) { //far shot
            outtake.setHoodAngle(.7);
            outtake.setTargetVel(1300);
        }
        else if (gamepad2.dpadRightWasReleased()){ //mid shot
            outtake.setHoodAngle(.5);
            outtake.setTargetVel(1000);
        }
        else if (gamepad2.dpadLeftWasReleased()) {
            outtake.setTargetVel(0);
        }

        telemetry.addLine(outtake.outtakeLog());

        String[] motif = spindexer.getCurrentMotif();
        if (motif != null) {
            String motifCaption = motif[0] + ", " + motif[1] + ", " + motif[2];
            telemetry.addLine(motifCaption);
        }
    }

    public void updateTelem () {
    }
}