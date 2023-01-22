//Property of FTC Team 22345 - All External users must request permission to access and utilize code
//Authors: Krish K. & Ronald Q.

package org.firstinspires.ftc.teamcode.test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.utility.OpenCV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class CameraTest extends LinearOpMode {


    DcMotorEx frontLeft;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotorEx frontRight;

    Claw claw = new Claw();

    Elevator elevator = new Elevator();

    OpenCvWebcam webcam;
    OpenCV.Pipeline pipeline;
    OpenCV.Pipeline.Orientation snapshotAnalysis = OpenCV.Pipeline.Orientation.PROCESSING; // default

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //Front Drive Motors Initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Back Drive Motors Initialization
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        //Claw Initialization
        claw.init(hardwareMap);
        boolean clawState = false; //closed

        //Elevator Initialization
        elevator.init(hardwareMap);
        int elevatorLevel = 0;

        //Camera Initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new OpenCV.Pipeline();
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
                telemetry.addData("Camera Initialization: ", "Success");
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Initialization: ", "Failed");
                telemetry.update();
            }
        });

        snapshotAnalysis = pipeline.getAnalysis();




        telemetry.addLine("Waiting for Robot Initialization...");
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {


            telemetry.addData("Robot Status: ", "Initialized");
            telemetry.addData("avg", pipeline.getColorAverage());
            telemetry.update();

            FtcDashboard.getInstance().startCameraStream(webcam, 0);

            if(gamepad1.a){
                webcam.stopStreaming();
            }


        }


    }


}
