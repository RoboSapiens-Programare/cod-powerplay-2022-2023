/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(group="Autonomous")

public class AutonomousBlue extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new Robot(hardwareMap);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            Pose2d start = new Pose2d(35, -60, Math.toRadians(0));
            robot.drive.setPoseEstimate(start);

            TrajectorySequence myTrajectory1 = robot.drive.trajectorySequenceBuilder(start)
                    .addDisplacementMarker(() -> {
                        robot.glisiera.strangeCleste();
                    })
                    .lineToLinearHeading(new Pose2d(35, -9, Math.toRadians(90)))
                    .build();

            TrajectorySequence myTrajectory2 = robot.drive.trajectorySequenceBuilder(myTrajectory1.end())
                    .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(90)))
                    .build();

            TrajectorySequence myTrajectory3 = robot.drive.trajectorySequenceBuilder(myTrajectory2.end())
                    .lineToLinearHeading(new Pose2d(35, -11, Math.toRadians(180)))
                    .build();

            TrajectorySequence myTrajectory4 = robot.drive.trajectorySequenceBuilder(myTrajectory3.end())
                    .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(180)))
                    .build();
            TrajectorySequence myTrajectory5 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .lineToLinearHeading(new Pose2d(24, -10, Math.toRadians(270)))
                    .build();

            TrajectorySequence myTrajectory6 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .lineToLinearHeading(new Pose2d(24, -14, Math.toRadians(270)))
                    .build();

            TrajectorySequence myTrajectory7 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .addDisplacementMarker(() -> {
                        robot.glisiera.manualLevel(1200);
                        sleep(2000);
                        robot.glisiera.desfaCleste();
                        sleep(2000);
                        robot.glisiera.manualLevel(520);
                    })
                    .build();

            TrajectorySequence myTrajectory8 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .lineToLinearHeading(new Pose2d(24, -12, Math.toRadians(270)))
                    .build();

            TrajectorySequence myTrajectory9 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(0)))
                    .build();

            TrajectorySequence myTrajectory10 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                    .addDisplacementMarker(() -> {
                                    robot.glisiera.strangeCleste();
                                    robot.glisiera.mediumLevel();
                    })
                    .build();

            TrajectorySequence myTrajectory11 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .lineToLinearHeading(new Pose2d(48, -12, Math.toRadians(0)))
                    .build();

            TrajectorySequence myTrajectory12 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .lineToLinearHeading(new Pose2d(47, -12, Math.toRadians(180)))
                    .build();

            TrajectorySequence myTrajectory13 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(180)))
                    .build();

            TrajectorySequence myTrajectory14 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .lineToLinearHeading(new Pose2d(24, -10, Math.toRadians(270)))
                    .build();

            TrajectorySequence myTrajectory15 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .lineToLinearHeading(new Pose2d(24, -14, Math.toRadians(270)))
                    .build();

            TrajectorySequence myTrajectory16 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .lineToLinearHeading(new Pose2d(24, -12, Math.toRadians(270)))
                    .build();

            TrajectorySequence myTrajectory17 = robot.drive.trajectorySequenceBuilder(myTrajectory4.end())
                    .addDisplacementMarker(() -> {
                                    robot.glisiera.manualLevel(1200);
                                    sleep(2000);
                                    robot.glisiera.desfaCleste();
                    })
                    .build();

            robot.drive.followTrajectorySequence(myTrajectory1);
            robot.drive.followTrajectorySequence(myTrajectory2);
            robot.drive.followTrajectorySequence(myTrajectory3);
            robot.drive.followTrajectorySequence(myTrajectory4);
            robot.drive.followTrajectorySequence(myTrajectory5);
            robot.drive.followTrajectorySequence(myTrajectory6);
            robot.drive.followTrajectorySequence(myTrajectory7);
            robot.drive.followTrajectorySequence(myTrajectory8);
            robot.drive.followTrajectorySequence(myTrajectory9);
            robot.drive.followTrajectorySequence(myTrajectory10);
            robot.drive.followTrajectorySequence(myTrajectory11);
            robot.drive.followTrajectorySequence(myTrajectory12);
            robot.drive.followTrajectorySequence(myTrajectory13);
            robot.drive.followTrajectorySequence(myTrajectory14);
            robot.drive.followTrajectorySequence(myTrajectory15);
            robot.drive.followTrajectorySequence(myTrajectory16);
            robot.drive.followTrajectorySequence(myTrajectory17);
        }





            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

