package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.limelight.Limelighter;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class StartRedBottom extends LinearOpMode {

    MecanumDrive mecanumDrive;
    Limelighter limelight;
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(60.0, 13 ,180);

        mecanumDrive = new MecanumDrive(this.hardwareMap, beginPose);
        limelight = new Limelighter(this.hardwareMap);

        waitForStart();

        TrajectoryActionBuilder goToShootingZone = mecanumDrive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(135)), Math.toRadians(0));


        Actions.runBlocking(
//                limelight.
                goToShootingZone
                //run
                        .build()
        );
    }
}
