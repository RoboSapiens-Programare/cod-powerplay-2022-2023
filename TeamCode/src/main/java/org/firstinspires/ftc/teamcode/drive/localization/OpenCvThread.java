package org.firstinspires.ftc.teamcode.drive.localization;

import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCvThread extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input)
    {
        return input;
    }

}
