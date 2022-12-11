package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.w3c.dom.html.HTMLAreaElement;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);
        final double FOAM_TILE_INCH = 24.0;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 68, Math.toRadians(180), Math.toRadians(180), 15.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -60, Math.toRadians(90)))
                                .addDisplacementMarker(() ->{
                                    //strange cleste
                                })
                                .lineToLinearHeading(new Pose2d(35, -9, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(35, -11, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(24, -10, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(24, -14, Math.toRadians(270)))
                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.manualLevel(1200);
//                                    robot.glisiera.desfaCleste();
//                                    robot.glisiera.manualLevel(520);
                                })
                                .lineToLinearHeading(new Pose2d(24, -12, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.strangeCleste();
//                                    robot.glisiera.mediumLevel();
                                })
                                .lineToLinearHeading(new Pose2d(48, -12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(47, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(24, -10, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(24, -14, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(24, -12, Math.toRadians(270)))
                                .addDisplacementMarker(() -> {
//                                    robot.glisiera.manualLevel(1200);
//                                    robot.glisiera.desfaCleste();
                                })
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}