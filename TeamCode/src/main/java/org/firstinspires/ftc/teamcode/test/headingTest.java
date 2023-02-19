package org.firstinspires.ftc.teamcode.test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Constants;

@Autonomous
public class headingTest extends LinearOpMode{
    Drive drivetrain = new Drive();

    @Override
    @SuppressLint("Default Locale")
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Waiting for Robot Initialization...");
        telemetry.update();
        drivetrain.init(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

        while (opModeInInit()) {
            drivetrain.resetIMU();
            telemetry.addData("heading", drivetrain.IMUheading());
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {
            //drivetrain.forward(500);
            drivetrain.forward(4000);
            drivetrain.backward(4000);
//            drivetrain.forward(500);
//            drivetrain.turnLeft(180);
//            drivetrain.forward(500);
//            drivetrain.turnRight(90);
//            drivetrain.backward(500);
        }

    }



}
