package org.firstinspires.ftc.teamcode.opModes;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.utility.OpenCV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class AutoOpBlueL extends LinearOpMode{
    Drive drivetrain = new Drive();

    @SuppressLint("Default Locale")
    @Override
    public void runOpMode(){
        telemetry = FtcDashboard.getInstance().getTelemetry();

        telemetry.addLine("Waiting for Robot Initialization...");
        telemetry.update();
        drivetrain.init(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

        while (opModeInInit()){
            drivetrain.resetIMU();
        }
        waitForStart();

//        while (opModeIsActive() && !isStopRequested())
//        {
//            telemetry.addData("Front Left: ", drivetrain.getFLPos());
//            telemetry.addData("Front Right: ", drivetrain.getFRPos());
//            telemetry.addData("Back Left: ", drivetrain.getBLPos());
//            telemetry.addData("Back Right: ", drivetrain.getBRPos());

            drivetrain.forward(400);



//            telemetry.update();
//            drivetrain.update();
            //drivetrain.reset();

//            break;

//        }

    }

}
