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

import static org.firstinspires.ftc.teamcode.drive.opmode.LinearDriveMode.TALL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.visualrecog.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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

@Autonomous(name = "Autonomie Tall Junction", group="autonomous")

public class AutonomousTall extends LinearOpMode {

    // Declare OpMode members.
    private Robot robot = null;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    public AprilTagDetection tagOfInterest = null;
    private ElapsedTime timer;
    private final int MAX_MILISECONDS = 1000;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new Robot(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
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

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
//                tagOfInterest.id = MIDDLE;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
            }
            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) return;

        while (opModeIsActive()){
            telemetry.addData("Tag:", tagOfInterest.id);

//            robot.outtake.strangeCleste();

            Pose2d start = new Pose2d(36, -60, Math.toRadians(90));
            robot.drive.setPoseEstimate(start);
//=======
//>>>>>>> Stashed changes

            TrajectorySequence Preload = robot.drive.trajectorySequenceBuilder(start)
                .forward(48)
                .turn(Math.toRadians(35))
                .addDisplacementMarker(()->{
                    robot.outtake.setLevel(TALL);
                    timer = new ElapsedTime();
                    timer.startTime();
                    while(timer.milliseconds() < MAX_MILISECONDS);
                })
                .forward(10)
                .addDisplacementMarker(()->{
                    robot.outtake.manualLevel(3000);
                    timer = new ElapsedTime();
                    timer.startTime();
                    while(timer.milliseconds() < MAX_MILISECONDS);
//                    robot.outtake.desfaCleste();
                })
                .back(10)
                .turn(Math.toRadians(-125))
                .forward(9)
                .build();
            robot.drive.followTrajectorySequence(Preload);

            Pose2d lastPose = Preload.end();


            for(int i = 1; i <= 5; i++){
//              robot.glisiera.manualLevel(ceva + i * altceva);
//                robot.outtake.strangeCleste();
                timer = new ElapsedTime();
                timer.startTime();
                while(timer.milliseconds() < MAX_MILISECONDS);
                TrajectorySequence Score = robot.drive.trajectorySequenceBuilder(lastPose)
                    .back(9)
                    .turn(Math.toRadians(125))
                    .addDisplacementMarker(()->{
                        robot.outtake.setLevel(TALL);
                        timer = new ElapsedTime();
                        timer.startTime();
                        while(timer.milliseconds() < MAX_MILISECONDS);
                    })
                    .forward(10)
                    .addDisplacementMarker(()->{
                        robot.outtake.manualLevel(3000);
                        timer = new ElapsedTime();
                        timer.startTime();
                        while(timer.milliseconds() < MAX_MILISECONDS);
//                        robot.outtake.desfaCleste();
                    })
                    .back(10)
                    .turn(Math.toRadians(-125))
                    .forward(9)
                    .build();
                robot.drive.followTrajectorySequence(Score);

                lastPose = Score.end();
            }

            TrajectorySequence park = robot.drive.trajectorySequenceBuilder(lastPose)
                    .back(24 * (3-tagOfInterest.id) + 1)
                    .build();
            robot.drive.followTrajectorySequence(park);

//            if (tagOfInterest.id == MIDDLE) {
//                robot.drive.setPoseEstimate(start);
//
//                TrajectorySequence parkMid = robot.drive.trajectorySequenceBuilder(start)
//                    .back(24)
//                    .build();
//
//                robot.drive.followTrajectorySequence(parkMid);
//            }
//            else if (tagOfInterest.id == LEFT) {
//                robot.drive.setPoseEstimate(start);
//
//                TrajectorySequence parkLeft = robot.drive.trajectorySequenceBuilder(start)
//                    .back(48)
//                    .build();
//                robot.drive.followTrajectorySequence(parkLeft);
//            }
//            else if (tagOfInterest.id == RIGHT) {
//                robot.drive.setPoseEstimate(start);
//
//                TrajectorySequence parkRight = robot.drive.trajectorySequenceBuilder(start)
//                        .back(2)
//                    .build();
//                robot.drive.followTrajectorySequence(parkRight);
//            }
        }
        telemetry.update();
    }
}


