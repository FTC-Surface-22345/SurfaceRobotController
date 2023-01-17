//Property of FTC Team 22345 - All External users must request permission to access and utilize code
//Authors: Krish K. & Ronald Q.

package org.firstinspires.ftc.teamcode.test;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utility.OpenCV;
import org.openftc.easyopencv.*;

@Autonomous
public class MotorPos extends LinearOpMode{


    DcMotorEx frontLeft;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotorEx frontRight;

    Claw mainClaw = new Claw();

    Elevator elevator = new Elevator();

    OpenCvWebcam webcam;
    OpenCV.Pipeline pipeline;
    OpenCV.Pipeline.Orientation snapshotAnalysis = OpenCV.Pipeline.Orientation.PROCESSING; // default

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        //Front Drive Motors Initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);



        //Back Drive Motors Initialization
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        //Claw Initialization
        mainClaw.init(hardwareMap);
        boolean clawState = false; //closed

        //Camera Initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new OpenCV.Pipeline();
        webcam.setPipeline(pipeline);

        ((OpenCvWebcam) webcam).setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override

            public void onOpened(){
                webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                webcam.startStreaming(1280,720,OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode){
                telemetry.addData("Camera Initialization: ", "Failed");
                telemetry.update();
            }
        });

        snapshotAnalysis = pipeline.getAnalysis();

        telemetry.addLine("Waiting for Robot Initialization...");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("avg", pipeline.getColorAverage());
            telemetry.addData("Front Left: ", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right: ", frontRight.getCurrentPosition());
            telemetry.addData("Back Left: ", backLeft.getCurrentPosition());
            telemetry.addData("Back Right: ", backRight.getCurrentPosition());
            telemetry.update();

            if(gamepad1.a){
                webcam.stopStreaming();
            }

            sleep(100);
        }

    }








}
