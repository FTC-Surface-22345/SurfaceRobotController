package org.firstinspires.ftc.teamcode.autonomous;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConvertToGrayPipeline extends OpenCvPipeline{
    Mat gray = new Mat();

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        return gray;
    }
}
