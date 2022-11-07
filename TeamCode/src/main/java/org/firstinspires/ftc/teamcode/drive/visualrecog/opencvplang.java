package org.firstinspires.ftc.teamcode.drive.visualrecog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Auto: SkyStone Detector")
public class opencvplang extends LinearOpMode{


        // Handle hardware stuff...

        int width = 320;
        int height = 240;
        // store as variable here so we can access the location
        EopencvPipeline detector = new EopencvPipeline(width);
        OpenCvCamera camera;

        @Override
        public void runOpMode() {
            // robot logic...

            // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
            // Initialize the back-facing camera
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);
            // Connect to the camera
            camera.openCameraDevice();
            // Use the SkystoneDetector pipeline
            // processFrame() will be called to process the frame
            camera.setPipeline(detector);
            // Remember to change the camera rotation
            camera.startStreaming(width, height);

            //...

            EopencvPipeline.ConeLocation location = detector.getLocation();
            if (location != EopencvPipeline.ConeLocation.NONE) {
                // Move to the left / right
            } else {
                // Grab the skystone
            }

            // more robot logic...
        }

    }

