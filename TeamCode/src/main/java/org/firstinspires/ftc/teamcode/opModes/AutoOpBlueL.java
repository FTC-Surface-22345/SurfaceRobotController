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
import org.firstinspires.ftc.teamcode.subsystems.Constants;

@Autonomous
public class AutoOpBlueL extends LinearOpMode {
    Drive drivetrain = new Drive();

    Claw claw = new Claw();

    Elevator elevator = new Elevator();

    Kamera kamera = new Kamera();

    @SuppressLint("Default Locale")
    @Override
    public void runOpMode() {
        telemetry = FtcDashboard.getInstance().getTelemetry();

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
            claw.moveClaw(Constants.clawState.CLOSE);
        }
        waitForStart();

//        while (opModeIsActive() && !isStopRequested()) {
        elevator.moveLift(Constants.elevatorPos.GJUNC);
        drivetrain.forward(1120);
        sleep(500);
        drivetrain.reset();
        sleep(500);
        drivetrain.strafeRight(1800);
        sleep(150);
        drivetrain.reset();
        sleep(150);
        elevator.moveLift(Constants.elevatorPos.HIGH);
        sleep(2000);
        drivetrain.forward(100);
        sleep(500);
        drivetrain.reset();
        sleep(200);
        claw.open();
        sleep(200);
        drivetrain.backward(70);
        sleep(500);
        drivetrain.reset();
        sleep(400);
        elevator.moveLift(Constants.elevatorPos.GROUND);

        //park
        switch(position){
            case 1:
                drivetrain.strafeLeft(3000);
                telemetry.addData("Position", position);
                break;
            case 2:
                drivetrain.strafeLeft(1750);
                telemetry.addData("Position", position);
                break;
            case 3:
                drivetrain.strafeLeft(500);
                telemetry.addData("Position", position);
                break;
            case -1:
                telemetry.addLine("Zone Failure");
                break;
        }





    }

}


