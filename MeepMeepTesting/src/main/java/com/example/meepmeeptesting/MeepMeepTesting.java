package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(34.4, -63.4, Math.toRadians(90)))

                                .lineToConstantHeading(new Vector2d(34.4,-11.4))
                                .turn(Math.toRadians(-139))
                                .forward(10)
                                .back(10)
                                .turn(Math.toRadians(-139))
                                .back(24)
                                .lineToConstantHeading(new Vector2d(60.4,-11.4))
                                .lineToConstantHeading(new Vector2d(34.4,-11.4))
                                .turn(Math.toRadians(139))
                                .forward(10)
                                .back(10)
                                .waitSeconds(2)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}