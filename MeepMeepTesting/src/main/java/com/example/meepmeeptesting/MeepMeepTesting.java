package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    private static RoadRunnerBotEntity myBot;
    private final Pose2d startRedTop = new Pose2d(new Vector2d(-60.0, -37), Math.toRadians(0));
    private final Pose2d startblueTop = new Pose2d(new Vector2d(-60.0, 37), Math.toRadians(0));
    private final Pose2d startBlueBottom = new Pose2d(new Vector2d(60.0, -13), 180);
    private final Pose2d startRedBottom = new Pose2d(new Vector2d(60.0, 13), 180);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // left is inverted on the field, so we may need to make all negatives x
        // coordinates and vectors in the commmands in actual testing this may mess with
        // tunining, y is fine,
        // heading should not be affected as its determined by what you initally set it
        // too
        // or we could reverse encoder direction on the x wheel
        // update: Y may need to be treated as x and vice versa

        // pathing,
        // Pos1 : go from starting position, to a little bit out so we can read the
        // obelisk for the motif
        //
        // Pos2 : go from motif reading position to the shooting position(75-80 inches
        // 3ish mats),
        // Pos3 :

        Pose2d beginPose = new Pose2d(new Vector2d(-60.0, 37), Math.toRadians(0));

        DriveShim mecanumDrive = myBot.getDrive();


        // start from goal go to center

        TrajectoryActionBuilder goToObelisk = mecanumDrive.actionBuilder(beginPose)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-34, 0, Math.toRadians(180)), Math.toRadians(10));

        TrajectoryActionBuilder goToShootingZone0 = mecanumDrive
                .actionBuilder(new Pose2d(-34, 0, Math.toRadians(180)))
                .turnTo(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-10, 10), Math.toRadians(135))
                .waitSeconds(1);

        TrajectoryActionBuilder goToBallSet1 = mecanumDrive
                .actionBuilder(new Pose2d(-10, 10, Math.toRadians(135)))
                .splineToLinearHeading(new Pose2d(-11, 30, Math.toRadians(90)), Math.toRadians(20));

        TrajectoryActionBuilder intakeBallSet1 = mecanumDrive
                .actionBuilder(new Pose2d(-11, 30, Math.toRadians(90)))
                .lineToY(50)
                .turnTo(Math.toRadians(180));

        TrajectoryActionBuilder goToShootingZone1 = mecanumDrive
                .actionBuilder(new Pose2d(-11, 50, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(135)), Math.toRadians(-10))
                .waitSeconds(1);

        TrajectoryActionBuilder goToBallSet2 = mecanumDrive
                .actionBuilder(new Pose2d(-10, 10, Math.toRadians(135)))
                .splineToLinearHeading(new Pose2d(12, 30, Math.toRadians(90)), Math.toRadians(-20));

        TrajectoryActionBuilder intakeBallSet2 = mecanumDrive
                .actionBuilder(new Pose2d(12, 30, Math.toRadians(90)))
                .lineToY(50);

        TrajectoryActionBuilder goToShootingZone2 = mecanumDrive
                .actionBuilder(new Pose2d(12, 50, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(135)), Math.toRadians(30))
                .waitSeconds(1);

        TrajectoryActionBuilder turnInPlace = mecanumDrive
                .actionBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .turn(Math.toRadians(360));

        Pose2d position = new Pose2d(0,0, Math.toRadians(0));

        // Build trajectory actions with updated positions
        com.acmerobotics.roadrunner.Action[] moveActions = new com.acmerobotics.roadrunner.Action[8];
        for (int i = 0; i < 8; i++) {
            TrajectoryActionBuilder moveBackwards = mecanumDrive
                    .actionBuilder(position)
                    .lineToX(position.position.x - 5); // move 5 inches
            moveActions[i] = moveBackwards.build();
            // Update position for next iteration
            position = new Pose2d(position.position.x - 5, position.position.y, position.heading.toDouble());
        }

        myBot.runAction(new SequentialAction(moveActions));

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    private static Pose2d getLastPose() {
        return myBot.getDrive().getPoseEstimate();
    }
}