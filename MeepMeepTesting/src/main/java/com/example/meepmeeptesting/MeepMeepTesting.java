package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {


    private Pose2d startRedTop = new  Pose2d(new Vector2d(-60.0, -37), Math.toRadians(0));
    private Pose2d startblueTop = new  Pose2d(new Vector2d(-60.0, 37), Math.toRadians(0));

    private Pose2d startBlueBottom = new Pose2d(new Vector2d(60.0, -13) ,180);
    private Pose2d startRedBottom = new Pose2d(new Vector2d(60.0, 13) ,180);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        //left is inverted on the field, so we may need to make all negatives x coordinates and vectors  in the commmands in actual testing this may mess with tunining, y is fine,
        // heading should not be affected as its determined by what you initally set it too
        //or we could reverse encoder direction on the x wheel

        //pathing,
        // Pos1 : go from starting position, to a little bit out so we can read the obelisk for the motif
        //
        // Pos2 : go from motif reading position to the shooting position(75-80 inches 3ish mats),
        // Pos3 :

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(60.0, -13) ,Math.toRadians(180))) // start from the blue goal
                .lineToX(59)
//                .lineToX(30)
//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}