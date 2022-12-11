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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.visualrecog.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


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

public class AutonomousTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot = null;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    final int LEFT = 1;
    final int MIDDLE = 2;
    final int RIGHT = 3;
    public AprilTagDetection tagOfInterest = null;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new Robot(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        telemetry.setMsTransmissionInterval(50);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
            }

            telemetry.update();
            sleep(20);

            Pose2d start = new Pose2d(32, -60, Math.toRadians(90));
            robot.drive.setPoseEstimate(start);

            TrajectorySequence myTrajectory = robot.drive.trajectorySequenceBuilder(start)
                    .lineToLinearHeading(new Pose2d(32, 0, Math.toRadians(0)))
                    .waitSeconds(2)
                    .lineToLinearHeading(new Pose2d(32, -12, Math.toRadians(-90)))
                    .waitSeconds(2)
                    .addDisplacementMarker(() -> {
                        robot.glisiera.desfaCleste();
                        robot.glisiera.manualLevel(520);
                    })
                    .waitSeconds(2)
                    .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                    .waitSeconds(2)
                    .addDisplacementMarker(() -> {
                        robot.glisiera.strangeCleste();
                        robot.glisiera.mediumLevel();
                    })
                    .waitSeconds(2)
                    .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(-90)))
                    .waitSeconds(2)
                    .lineToLinearHeading(new Pose2d(24, -14, Math.toRadians(-90)))
                    .waitSeconds(2)
                    .addDisplacementMarker(() -> {
                        robot.glisiera.manualLevel(1200);
                        robot.glisiera.desfaCleste();
                    })
                    .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(-90)))
                    .build();
            robot.drive.followTrajectorySequence(myTrajectory);

            start = new Pose2d(24, -11, Math.toRadians(-90));

            if(tagOfInterest == null || tagOfInterest.id == LEFT){
                /**Trajectory 1:*/

                myTrajectory = robot.drive.trajectorySequenceBuilder(start)
                    .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(-90)))
                    .build();
            }
            else if(tagOfInterest.id == MIDDLE){
                /**trajectory 2:*/

                myTrajectory = robot.drive.trajectorySequenceBuilder(start)
                        .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(-90)))
                        .build();
            }
            else if(tagOfInterest.id == RIGHT){
                /**trajectory 3:*/

                myTrajectory = robot.drive.trajectorySequenceBuilder(start)
                        .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(-90)))
                        .build();
            }

            robot.drive.followTrajectorySequence(myTrajectory);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
