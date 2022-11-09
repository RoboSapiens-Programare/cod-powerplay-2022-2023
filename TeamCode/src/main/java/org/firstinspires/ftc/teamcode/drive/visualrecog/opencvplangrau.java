package org.firstinspires.ftc.teamcode.drive.visualrecog;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.visualrecog.pipelines.opencvplangraudetotpipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
@Autonomous(group = "drive")
public class opencvplangrau extends LinearOpMode{
    static OpenCvWebcam Webcam;
    static opencvplangraudetotpipeline pipeline;

    private int cone;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Camera"); 
        Webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new opencvplangraudetotpipeline();
        Webcam.setPipeline(pipeline);

        Webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        FtcDashboard.getInstance().startCameraStream(Webcam, 0);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("exista", pipeline.isExist());
            telemetry.update();
        }


    }

}
