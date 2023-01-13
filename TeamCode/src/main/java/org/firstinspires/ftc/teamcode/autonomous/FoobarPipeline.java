package org.firstinspires.ftc.teamcode.autonomous;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class FoobarPipeline extends OpenCvPipeline{
    int lastResult = 0;

    @Override
    public Mat processFrame(Mat input){
        if(){
            lastResult = 1;
        }
        else if(){
            lastResult = 2;
        }
        else if(){
            lastResult = 3;
        }
    }

    public int getLatestResults{
        return lastResult;
    }
}
