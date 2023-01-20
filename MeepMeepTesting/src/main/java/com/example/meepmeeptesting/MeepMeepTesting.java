package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
                        drive.trajectorySequenceBuilder(new Pose2d(36, -60, Math.toRadians(90)))
//                                .lineToLinearHeading(new Pose2d(36, -12,Math.toRadians(215)))
                                .forward(55)
                                .back(7)
                                .turn(Math.toRadians(125))
//                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.mediumLevel();
//                                    ElapsedTime time = new ElapsedTime();
//                                    while(time.milliseconds() < MAX_MILISECONDS);
//                                })
                                .forward(10)
                                .waitSeconds(0.2)
//                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.manualLevel(1750);
//                                    ElapsedTime time = new ElapsedTime();
//                                    while(time.milliseconds() < MAX_MILISECONDS);
//                                })
//                                .waitSeconds(1)
//                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.desfaCleste();
//                                })
                                .waitSeconds(0.7)
                                .back(10)
//                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.zeroLevel();
//                                })
                                .turn(Math.toRadians(143))
                                .forward(26)
                                .back(26)
                                .turn(Math.toRadians(-143))

                                .forward(10)
                                .waitSeconds(0.2)
//                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.manualLevel(1750);
//                                    ElapsedTime time = new ElapsedTime();
//                                    while(time.milliseconds() < MAX_MILISECONDS);
//                                })
//                                .waitSeconds(1)
//                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.desfaCleste();
//                                })
                                .waitSeconds(0.7)
                                .back(10)
//                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.zeroLevel();
//                                })
                                .turn(Math.toRadians(143))
                                .forward(24)
                                .waitSeconds(2)
                                .back(24)
                                .turn(Math.toRadians(-143))

                                .forward(10)
                                .waitSeconds(0.2)
//                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.manualLevel(1750);
//                                    ElapsedTime time = new ElapsedTime();
//                                    while(time.milliseconds() < MAX_MILISECONDS);
//                                })
//                                .waitSeconds(1)
//                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.desfaCleste();
//                                })
                                .waitSeconds(0.7)
                                .back(10)
//                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.zeroLevel();
//                                })
                                .turn(Math.toRadians(143))
                                .forward(26)
                                .back(26)
                                .turn(Math.toRadians(-143))
//                                .lineToLinearHeading(new Pose2d(56,-12,Math.toRadians(358)))
//                                .waitSeconds(45)
//                                .waitSeconds(45)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}