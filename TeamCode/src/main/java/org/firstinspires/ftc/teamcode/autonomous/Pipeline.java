package org.firstinspires.ftc.teamcode.autonomous;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline{
    @Override
    public Mat processFrame(Mat input){
        return input;
    }
}