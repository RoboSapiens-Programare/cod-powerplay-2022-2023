package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.FOAM_TILE_INCH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.visualrecog.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;


@TeleOp(group = "autonomous")
public class AutonomousModeRosuDreapta extends LinearOpMode {
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
        PhotonCore.enable();
        telemetry.addData(">", "Initializing...");
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
        FtcDashboard.getInstance().startCameraStream(camera, 0);


        while (robot.isInitialize() && opModeIsActive()) {

            telemetry.addData(">", "Initialized");
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            while (opModeIsActive()) {
                idle();
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

                    if(tagFound)
                    {
                        telemetry.addData("Tag of interest is in sight!\n\nLocation data:", tagOfInterest.id);

                    }
                    else {
                        telemetry.addLine("Don't see tag of interest :(");
                    }
                    telemetry.update();
                    sleep(20);
                }

                Pose2d start = new Pose2d(32, -60, Math.toRadians(90));
                robot.drive.setPoseEstimate(start);

                TrajectorySequence myTrajectory = robot.drive.trajectorySequenceBuilder(start)
                        .lineToLinearHeading(new Pose2d(32, 0, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(32, -12, Math.toRadians(-90)))
                        .addDisplacementMarker(() -> {
                            robot.glisiera.desfaCleste();
                            robot.glisiera.manualLevel(520);
                        })
                        .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                        .addDisplacementMarker(() -> {
                            robot.glisiera.strangeCleste();
                            robot.glisiera.mediumLevel();
                        })

                        .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(-90)))
                        .forward(1)
                        .addDisplacementMarker(() -> {
                            robot.glisiera.manualLevel(1200);
                            robot.glisiera.desfaCleste();
                        })
                        .build();
                robot.drive.followTrajectorySequence(myTrajectory);

                if(tagOfInterest.id == LEFT){
                    myTrajectory = robot.drive.trajectorySequenceBuilder(new Pose2d())

                            .build();
                }
                else if(tagOfInterest.id == MIDDLE){
                    //trajectory
                }
                else {
                    //trajectory
                }
        }
    }

}
}
