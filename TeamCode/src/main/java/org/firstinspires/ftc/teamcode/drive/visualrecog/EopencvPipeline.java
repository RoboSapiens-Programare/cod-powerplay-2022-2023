package org.firstinspires.ftc.teamcode.drive.visualrecog;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class EopencvPipeline extends OpenCvPipeline{
    enum ConeLocation {
        LEFT,
        RIGHT,
        NONE
    }
    private int width;
    ConeLocation location;
    public EopencvPipeline(int width){
        this.width = width;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (mat.empty()) {
            location = ConeLocation.NONE;
            return input;
        }

        Scalar lowHSV = new Scalar(3, 71, 88);
        Scalar highHSV = new Scalar(7, 92, 55);
        Mat thresh = new Mat();
        Core.inRange(mat, lowHSV, highHSV, thresh);
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i <contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i],3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }
        double left_x = 0.25 * width;
        double right_x = 0.75 * width;
        boolean left = false; // true if regular stone found on the left side
        boolean right = false; // "" "" on the right side
        for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x < left_x)
                left = true;
            if (boundRect[i].x + boundRect[i].width > right_x)
                right = true;

            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
        }

        // if there is no yellow regions on a side
        // that side should be a Skystone
        if (!left) location = ConeLocation.LEFT;
        else if (!right) location = ConeLocation.RIGHT;
            // if both are true, then there's no Skystone in front.
            // since our team's camera can only detect two at a time
            // we will need to scan the next 2 stones
        else location = ConeLocation.NONE;

        return mat; // return the mat with rectangles drawn
    }

    public ConeLocation getLocation() {
        return this.location;
    }
    }


