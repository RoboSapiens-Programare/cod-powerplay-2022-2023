package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.w3c.dom.html.HTMLAreaElement;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        final double FOAM_TILE_INCH = 24.0;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 68, Math.toRadians(180), Math.toRadians(180), 15.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -60, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(0)))
                                .addDisplacementMarker(() -> {

                                })
                                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                                .addDisplacementMarker(() -> {

                                })

                                .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(-90)))
                                .forward(1)
                                .addDisplacementMarker(() -> {

                                })
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}