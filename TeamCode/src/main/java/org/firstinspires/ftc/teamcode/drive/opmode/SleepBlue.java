package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.visualrecog.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Autonomie sleep blue", group = "autonomous")
public class SleepBlue extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        PhotonCore.enable();
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

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        FtcDashboard.getInstance().startCameraStream(camera, 0);

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

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            robot.glisiera.strangeCleste();
//            robot.drive.mergi(700, new Pose2d(0.5, 0, 0));
//            robot.drive.mergi(300, new Pose2d(-0.3, 0, 0));
//            robot.drive.turn(Math.toRadians(90));
//            robot.drive.mergi(500, new Pose2d(0.5, 0, 0));
//            robot.glisiera.mediumLevel();
//            robot.drive.turn(Math.toRadians(90));
//            robot.drive.mergi(100, new Pose2d(0.5, 0, 0));
//            robot.glisiera.manualLevel(1200);
//            robot.glisiera.desfaCleste();
//            robot.drive.mergi(100, new Pose2d(-0.5, 0, 0));
//
            if(tagOfInterest == null || tagOfInterest.id == MIDDLE) {
//                robot.drive.turn(Math.toRadians(90));
//                robot.drive.mergi(250, new Pose2d(0.5, 0, 0));
//                robot.drive.turn(Math.toRadians(-90));
//                robot.drive.mergi(500, new Pose2d(0.5, 0, 0));
            }
            else if(tagOfInterest.id == LEFT){
//                robot.drive.turn(Math.toRadians(-90));
//                robot.drive.mergi(250, new Pose2d(0.5, 0, 0));
//                robot.drive.turn(Math.toRadians(90));
//                robot.drive.mergi(500, new Pose2d(0.5, 0, 0));
                robot.drive.turn(Math.toRadians(90));
                robot.drive.mergi(500, new Pose2d(0.5, 0, 0));
            }
            else {
                robot.drive.turn(Math.toRadians(-90));
                robot.drive.mergi(500, new Pose2d(0.5, 0, 0));
                robot.drive.turn(Math.toRadians(90));
                robot.drive.mergi(500, new Pose2d(0.5, 0, 0));
            }
            robot.glisiera.desfaCleste();
        }

        telemetry.update();
        sleep(20);
    }
}
