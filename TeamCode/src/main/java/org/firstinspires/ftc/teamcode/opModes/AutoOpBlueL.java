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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Waiting for Robot Initialization...");
        telemetry.update();
        drivetrain.init(hardwareMap);

        waitForStart();


        drivetrain.forward(4000);
        while (opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Front Left: ", drivetrain.getFLPos());
            telemetry.addData("Front Right: ", drivetrain.getFRPos());
            telemetry.addData("Back Left: ", drivetrain.getBLPos());
            telemetry.addData("Back Right: ", drivetrain.getBRPos());

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", drivetrain.zYaw());
            telemetry.addData("Pitch (X)", "%.2f Deg.", drivetrain.xPitch());
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", drivetrain.yRoll());

            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", drivetrain.YawVel());
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", drivetrain.PitchVel());
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", drivetrain.RollVel());

            telemetry.update();
            drivetrain.update();
            //drivetrain.reset();

        }

    }

}
