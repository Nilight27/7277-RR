package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, 72, (3*Math.PI)/2))
                .lineToY(55)
                .setTangent(0)
                .lineToXLinearHeading(55, (5*Math.PI)/4)
                .lineToXLinearHeading(48,(3*Math.PI)/2)
                .setTangent((3*Math.PI)/2)
                .lineToY(35)
                .splineToLinearHeading(new Pose2d(55,55,(5*Math.PI/4)),0)
                .splineToLinearHeading(new Pose2d(59,37,(3*Math.PI)/2),0)
                .splineToLinearHeading(new Pose2d(55,55,(5*Math.PI/4)),0)
                .lineToXLinearHeading(48,(3*Math.PI)/2)
                .setTangent((3*Math.PI)/2)
                .lineToY(15)
                .turnTo(Math.PI)
                .setTangent(0)
                .lineToX(25)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}