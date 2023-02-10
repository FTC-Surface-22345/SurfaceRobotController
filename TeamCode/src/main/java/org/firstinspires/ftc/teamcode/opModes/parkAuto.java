package org.firstinspires.ftc.teamcode.opModes;


import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Kamera;
import org.firstinspires.ftc.teamcode.subsystems.dashConstants;

@Autonomous
public class parkAuto extends LinearOpMode{
    Drive drivetrain = new Drive();

    Claw claw = new Claw();

    Elevator elevator = new Elevator();

    Kamera kamera = new Kamera();

    @SuppressLint("Default Locale")
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Waiting for Robot Initialization...");
        telemetry.update();
        drivetrain.init(hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        claw.init(hardwareMap);
        elevator.init(hardwareMap);
        kamera.init(hardwareMap);

        int position = 0;
        while (opModeInInit()) {
            position = kamera.getPosition();
            telemetry.addData("position", position);

            telemetry.update();
            sleep(50);
            drivetrain.resetIMU();
            claw.moveClaw(dashConstants.clawState.CLOSE);
        }
        waitForStart();

        drivetrain.forward(1120);
        sleep(500);
        drivetrain.reset();
        sleep(500);
        //park
        switch(position){
            case 1:
                drivetrain.strafeLeft(1300);
                telemetry.addData("Position", position);
                break;
            case 2:

                break;
            case 3:
                drivetrain.strafeRight(1250);
                telemetry.addData("Position", position);
                break;
            case -1:
                telemetry.addLine("Zone Failure");
                break;
        }
    }
}
