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

import static org.firstinspires.ftc.teamcode.drive.opmode.AutonomousBlueSperMergCopie.somn;
import static org.firstinspires.ftc.teamcode.drive.opmode.LinearDriveMode.LOW;
import static org.firstinspires.ftc.teamcode.drive.opmode.LinearDriveMode.MEDIUM;
import static org.firstinspires.ftc.teamcode.drive.opmode.LinearDriveMode.ZERO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous(name = "Autonomous final", group="autonomous")

public class AutonomousFinal extends LinearOpMode {

    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
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
    private final int MAX_MILISECONDS = 1500;
    private final static int ZERO = 0, GROUND = 100, LOW = 900, MEDIUM = 1550, TALL = 2100;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new Robot(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

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

        while (opModeIsActive()) {
            robot.intake.servoCleste2.setPosition(0);
            robot.intake.servoY.setPosition(0.5);
            robot.intake.setSlidePosition(300);
            robot.outtake.desfaCleste();
            telemetry.addData("Tag:", tagOfInterest.id);
                robot.outtake.desfaCleste();
            Pose2d start = new Pose2d(34.4, -63.4, Math.toRadians(90));
            robot.drive.setPoseEstimate(start);
            TrajectorySequence Preload = robot.drive.trajectorySequenceBuilder(start)
                    //mers in fata
                    .lineToConstantHeading(new Vector2d(34.4,-11.4))
                    .turn(Math.toRadians(47))
                    //pus pe pole
                    .back(2)
                    .addTemporalMarker(() -> {
                        robot.outtake.setLevel(TALL);
                    })
                    .waitSeconds(0.3)
                    .forward(12)
                    .waitSeconds(0.3)
                    .addTemporalMarker(() -> {
                        robot.outtake.setLevel(TALL-400);
                    })
                    .waitSeconds(0.1)
                    .addTemporalMarker(() -> {
                        robot.outtake.strangeCleste();
                    })
                    .waitSeconds(0.1)
                    .back(12)
                    .waitSeconds(0.2)
                    .addTemporalMarker(() -> {
                        robot.outtake.setLevel(0);
                    })
                    //luat con din stack
                    .turn(Math.toRadians(48))
                    .waitSeconds(0.2)
                    .addTemporalMarker(() -> {
                        robot.intake.firstIntakeSequence();
                        telemetry.update();
                    })
                    .waitSeconds(0.2)
                    .addTemporalMarker(() -> {
                       robot.intake.secondIntakeSequence();
                    })
                    .waitSeconds(0.4)
                    .addTemporalMarker(() -> {
                        robot.outtake.desfaCleste();
                    })
                    .addTemporalMarker(() -> {
                        robot.intake.servoCleste2.setPosition(0);
                        robot.intake.servoY.setPosition(0.5);
                        somn(600);
                        robot.intake.setSlidePosition(300);
                    })
                    .turn(Math.toRadians(-49))
                    .back(2)
                    //pus pe pole
                    .waitSeconds(0.2)
                    .addTemporalMarker(() -> {
                        robot.outtake.setLevel(TALL);
                    })
                    .waitSeconds(0.3)
                    .forward(12)
                    .waitSeconds(0.2)
                    .addTemporalMarker(() -> {
                        robot.outtake.setLevel(TALL-400);
                    })
                    .waitSeconds(0.1)
                    .addTemporalMarker(() -> {
                        robot.outtake.strangeCleste();
                    })
                    .waitSeconds(0.1)
                    .back(10)
                    .addTemporalMarker(() -> {
                        robot.outtake.setLevel(0);
                    })
                    //luat din stack
                    .turn(Math.toRadians(48))
                    .waitSeconds(0.2)
                    .addTemporalMarker(() -> {
                        robot.intake.firstIntakeSequence();
                        telemetry.update();
                    })
                    .waitSeconds(0.2)
                    .addTemporalMarker(() -> {
                        robot.intake.secondIntakeSequence();
                    })
                    .waitSeconds(0.4)
                    .addTemporalMarker(() -> {
                        robot.outtake.desfaCleste();
                    })
                    .addTemporalMarker(() -> {
                        robot.intake.servoCleste2.setPosition(0);
                        robot.intake.servoY.setPosition(0.5);
                        somn(600);
                        robot.intake.setSlidePosition(300);
                    })
                    .turn(Math.toRadians(-49))
                    .back(2)
                    //pus pe pole
                    .waitSeconds(0.2)
                    .addTemporalMarker(() -> {
                        robot.outtake.setLevel(TALL);
                    })
                    .waitSeconds(0.3)
                    .forward(12)
                    .waitSeconds(0.2)
                    .addTemporalMarker(() -> {
                        robot.outtake.setLevel(TALL-400);
                    })
                    .waitSeconds(0.1)
                    .addTemporalMarker(() -> {
                        robot.outtake.strangeCleste();
                    })
                    .waitSeconds(0.1)
                    .back(10)
                    .addTemporalMarker(() -> {
                        robot.outtake.setLevel(0);
                    })
                    .waitSeconds(10000)
                    .build();
            robot.drive.followTrajectorySequence(Preload);


            if (tagOfInterest.id == MIDDLE) {

            }
         else if (tagOfInterest.id == LEFT) {

        }
            else if (tagOfInterest.id == RIGHT) {

        }

        stop();
        //            telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
}




