//Property of FTC Team 22345 - All External users must request permission to access and utilize code
//Authors: Krish K. & Ronald Q.

package org.firstinspires.ftc.teamcode.opModes;

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
public class AutoOpSurface extends LinearOpMode {


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

        //Autonomous Zone
        int zone;

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

        ((OpenCvWebcam) webcam).setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Initialization: ", "Failed");
                telemetry.update();
            }
        });

        snapshotAnalysis = pipeline.getAnalysis();

        if (pipeline.getColorAverage() < 120) {
            zone = 1;
        }
        else if (pipeline.getColorAverage() >= 130){
            zone = 2;
        }
        else{
            zone = 3;
        }

        telemetry.addLine("Waiting for Robot Initialization...");
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            claw.closeServo();

            telemetry.addData("Robot Status: ", "Initialized");
            telemetry.addData("avg", pipeline.getColorAverage());
            telemetry.addData("Front Left: ", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right: ", frontRight.getCurrentPosition());
            telemetry.addData("Back Left: ", backLeft.getCurrentPosition());
            telemetry.addData("Back Right: ", backRight.getCurrentPosition());
            telemetry.update();

            while (frontLeft.getCurrentPosition() > -1160) {
                frontLeft.setTargetPosition(-1160);
                frontRight.setTargetPosition(-1160);
                backLeft.setTargetPosition(1160);
                backRight.setTargetPosition(-1160);

                frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                frontLeft.setPower(0.8);
                frontRight.setPower(0.8);
                backLeft.setPower(0.8);
                backRight.setPower(0.8);

                telemetry.addData("avg", pipeline.getColorAverage());
                telemetry.addData("Front Left: ", frontLeft.getCurrentPosition());
                telemetry.addData("Front Right: ", frontRight.getCurrentPosition());
                telemetry.addData("Back Left: ", backLeft.getCurrentPosition());
                telemetry.addData("Back Right: ", backRight.getCurrentPosition());
                telemetry.update();
            }

            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1500);


            while (frontLeft.getCurrentPosition() > -1900) {
                frontLeft.setTargetPosition(-1900);
                frontRight.setTargetPosition(1900);
                backLeft.setTargetPosition(-1900);
                backRight.setTargetPosition(-1900);

                frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                frontLeft.setPower(0.4);
                frontRight.setPower(0.4);
                backLeft.setPower(0.4);
                backRight.setPower(0.4);

                telemetry.addData("avg", pipeline.getColorAverage());
                telemetry.addData("Front Left: ", frontLeft.getCurrentPosition());
                telemetry.addData("Front Right: ", frontRight.getCurrentPosition());
                telemetry.addData("Back Left: ", backLeft.getCurrentPosition());
                telemetry.addData("Back Right: ", backRight.getCurrentPosition());
                telemetry.update();
            }

            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1200);

            elevator.setLiftPosition(4180);

            sleep(2000);

            while (frontLeft.getCurrentPosition() > -180) {
                frontLeft.setTargetPosition(-180);
                frontRight.setTargetPosition(-180);
                backLeft.setTargetPosition(180);
                backRight.setTargetPosition(-180);

                frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                frontLeft.setPower(0.4);
                frontRight.setPower(0.4);
                backLeft.setPower(0.4);
                backRight.setPower(0.4);

                telemetry.addData("avg", pipeline.getColorAverage());
                telemetry.addData("Front Left: ", frontLeft.getCurrentPosition());
                telemetry.addData("Front Right: ", frontRight.getCurrentPosition());
                telemetry.addData("Back Left: ", backLeft.getCurrentPosition());
                telemetry.addData("Back Right: ", backRight.getCurrentPosition());
                telemetry.update();
            }

            sleep(2000);

            claw.openServo();


            telemetry.addData("Run", "Reached");

            sleep(500);

            //while (frontLeft.getCurrentPosition() < 180) {
            frontLeft.setTargetPosition(180);
            frontRight.setTargetPosition(180);
            backLeft.setTargetPosition(-180);
            backRight.setTargetPosition(180);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.4);
            frontRight.setPower(0.4);
            backLeft.setPower(0.4);
            backRight.setPower(0.4);


            telemetry.addData("avg", pipeline.getColorAverage());
            telemetry.addData("Front Left: ", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right: ", frontRight.getCurrentPosition());
            telemetry.addData("Back Left: ", backLeft.getCurrentPosition());
            telemetry.addData("Back Right: ", backRight.getCurrentPosition());
            telemetry.update();
            //}

            sleep(500);

            elevator.setLiftPosition(50);

            sleep(4000);
//
//            if (zone == 1){
//                while (frontLeft.getCurrentPosition() < 800) {
//                    frontLeft.setTargetPosition(800);
//                    frontRight.setTargetPosition(-800);
//                    backLeft.setTargetPosition(800);
//                    backRight.setTargetPosition(800);
//
//                    frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//                    frontLeft.setPower(0.4);
//                    frontRight.setPower(0.4);
//                    backLeft.setPower(0.4);
//                    backRight.setPower(0.4);
//
//                    telemetry.addData("avg", pipeline.getColorAverage());
//                    telemetry.addData("Front Left: ", frontLeft.getCurrentPosition());
//                    telemetry.addData("Front Right: ", frontRight.getCurrentPosition());
//                    telemetry.addData("Back Left: ", backLeft.getCurrentPosition());
//                    telemetry.addData("Back Right: ", backRight.getCurrentPosition());
//                    telemetry.update();
//                }
//            }
//
//            if (zone == 2){
//                while (frontLeft.getCurrentPosition() < 1900) {
//                    frontLeft.setTargetPosition(1900);
//                    frontRight.setTargetPosition(-1900);
//                    backLeft.setTargetPosition(1900);
//                    backRight.setTargetPosition(1900);
//
//                    frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//                    frontLeft.setPower(0.4);
//                    frontRight.setPower(0.4);
//                    backLeft.setPower(0.4);
//                    backRight.setPower(0.4);
//
//                    telemetry.addData("avg", pipeline.getColorAverage());
//                    telemetry.addData("Front Left: ", frontLeft.getCurrentPosition());
//                    telemetry.addData("Front Right: ", frontRight.getCurrentPosition());
//                    telemetry.addData("Back Left: ", backLeft.getCurrentPosition());
//                    telemetry.addData("Back Right: ", backRight.getCurrentPosition());
//                    telemetry.update();
//                }
//            }
//
//            if (zone == 3){
//                while (frontLeft.getCurrentPosition() < 2600) {
//                    frontLeft.setTargetPosition(2600);
//                    frontRight.setTargetPosition(-2600);
//                    backLeft.setTargetPosition(2600);
//                    backRight.setTargetPosition(2600);
//
//                    frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//                    frontLeft.setPower(0.4);
//                    frontRight.setPower(0.4);
//                    backLeft.setPower(0.4);
//                    backRight.setPower(0.4);
//
//                    telemetry.addData("avg", pipeline.getColorAverage());
//                    telemetry.addData("Front Left: ", frontLeft.getCurrentPosition());
//                    telemetry.addData("Front Right: ", frontRight.getCurrentPosition());
//                    telemetry.addData("Back Left: ", backLeft.getCurrentPosition());
//                    telemetry.addData("Back Right: ", backRight.getCurrentPosition());
//                    telemetry.update();
//                }
//            }

            break;


        }


    }


}
