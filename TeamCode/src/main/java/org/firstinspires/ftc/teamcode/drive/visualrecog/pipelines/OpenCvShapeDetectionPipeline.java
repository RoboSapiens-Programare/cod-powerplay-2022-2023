package org.firstinspires.ftc.teamcode.drive.visualrecog.pipelines;




import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCvShapeDetectionPipeline extends OpenCvPipeline {
    Mat mat = new Mat();
    public String cone = "0";

    private final Scalar LOW_RED = new Scalar(6, 99, 19);
    private final Scalar HIGH_RED = new Scalar(352, 37, 100);

    private static final double PERCENT_COLOR_THRESHOLD_1 = 0.3;

    static final Rect screen = new Rect(
            new Point(0, 0),
            new Point(320, 240)
    );

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = LOW_RED;
        Scalar highHSV = HIGH_RED;

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat center = mat.submat(screen);
        final Scalar BLUE = new Scalar(0, 0, 255);
        Imgproc.rectangle(input, screen, BLUE, 2);

        double value = Core.sumElems(center).val[0] / screen.area() / 255;

        center.release();

        boolean isCone1 = value > PERCENT_COLOR_THRESHOLD_1;


        if (isCone1)
        {
//            telemetry.addLine("cone1");
            cone = "1";
        }
        else
        {
//            telemetry.addLine("nu e boss");
            cone = "0";
        }

//        telemetry.update();


        return input;
    }

    public String isExist(){
        return cone;
    }

}
