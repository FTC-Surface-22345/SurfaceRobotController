package org.firstinspires.ftc.teamcode.test;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.utility.PID;

import org.firstinspires.ftc.teamcode.subsystems.dashConstants;



@TeleOp
public class IMUtest extends LinearOpMode {
    BNO055IMU imu;

    Orientation angles;
    float IMUheading;
    float offsetAngle;


    @SuppressLint("Default Locale")
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Waiting for Robot Initialization...");
        telemetry.update();

        while (opModeInInit()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            offsetAngle = angles.firstAngle;
            telemetry.addData("heading", angles.firstAngle);
            telemetry.addData("roll", angles.secondAngle);
            telemetry.addData("pitch", angles.thirdAngle);
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){
            IMUheading = angles.firstAngle - offsetAngle;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading", angles.firstAngle);
            telemetry.addData("roll", angles.secondAngle);
            telemetry.addData("pitch", angles.thirdAngle);
            telemetry.addData("IMUheading", IMUheading);
            telemetry.addData("offset", offsetAngle);
            telemetry.update();
        }
    }
}
