package com.example.meepmeeptesting;

import static com.acmerobotics.roadrunner.Actions.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {


    private Pose2d startRedTop = new  Pose2d(new Vector2d(-60.0, -37), Math.toRadians(0));
    private Pose2d startblueTop = new  Pose2d(new Vector2d(-60.0, 37), Math.toRadians(0));

    private Pose2d startBlueBottom = new Pose2d(new Vector2d(60.0, -13) ,180);
    private Pose2d startRedBottom = new Pose2d(new Vector2d(60.0, 13) ,180);

    private static RoadRunnerBotEntity myBot;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

         myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        //left is inverted on the field, so we may need to make all negatives x coordinates and vectors  in the commmands in actual testing this may mess with tunining, y is fine,
        // heading should not be affected as its determined by what you initally set it too
        //or we could reverse encoder direction on the x wheel
        //update: Y may need to be treated as x and vice versa

        //pathing,
        // Pos1 : go from starting position, to a little bit out so we can read the obelisk for the motif
        //
        // Pos2 : go from motif reading position to the shooting position(75-80 inches 3ish mats),
        // Pos3 :

        Pose2d beginPose = new Pose2d(new Vector2d(-60.0, -37), Math.toRadians(0));

        // start from goal go to center

        TrajectoryActionBuilder goToObelisk = myBot.getDrive().actionBuilder(new  Pose2d(new Vector2d(-60.0, -37), Math.toRadians(270)))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(-34, 0, Math.toRadians(180)), Math.toRadians(45) // may have to go back futher like 24 or 20
        );



        TrajectoryActionBuilder goToShootingZone = myBot.getDrive().actionBuilder(new Pose2d(-34, 0, Math.toRadians(180)))
                .waitSeconds(1)
                .turnTo(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(135)), Math.toRadians(0)

                );

        TrajectoryActionBuilder test = myBot.getDrive().actionBuilder(new Pose2d(-60.0, 37, Math.toRadians(0)))
                .waitSeconds(1)
                .lineToX(-50
                );

        myBot.runAction(
                goToObelisk.build()
        );

//
//        myBot.runAction(
//                goToShootingZone.build()
//        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    private static Pose2d getLastPose () {
        return myBot.getDrive().getPoseEstimate();
    }
}