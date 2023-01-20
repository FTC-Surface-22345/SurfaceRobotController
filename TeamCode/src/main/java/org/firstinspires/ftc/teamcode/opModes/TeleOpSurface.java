//Property of FTC Team 22345 - All External users must request permission to access and utilize code
//Authors: Krish K. & Ronald Q.

package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

@TeleOp(name = "TeleOpSurface")
public class TeleOpSurface extends LinearOpMode {

    DcMotorEx frontLeft;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotorEx frontRight;

    Claw claw = new Claw();

    Elevator elevator = new Elevator();


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //Front Drive Motors Initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp

        //Back Drive Motors Initialization
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp

        //Claw Initialization
        claw.init(hardwareMap);
        boolean clawState = true; //closed

        //Elevator Initialization
        elevator.init(hardwareMap);
        int elevatorLevel = 0;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Robot Status", "Initialized");
                telemetry.addData("Elevator Position: ", elevator.getPosition());
                telemetry.addData("Claw State: ", clawState);
                telemetry.addData("Front Left: ", frontLeft.getCurrentPosition());
                telemetry.addData("Front Right: ", frontRight.getCurrentPosition());
                telemetry.addData("Back Left: ", backLeft.getCurrentPosition());
                telemetry.addData("Back Right: ", backRight.getCurrentPosition());
                telemetry.update();

                backLeft.setPower((gamepad1.left_stick_y + (gamepad1.left_stick_x - gamepad1.right_stick_x)) / 1.3);
                frontLeft.setPower((gamepad1.left_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x)) / 1.3);
                backRight.setPower((gamepad1.left_stick_y - (gamepad1.left_stick_x - gamepad1.right_stick_x)) / 1.3);
                frontRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / 1.3);

                //Close Claw
                 if (gamepad1.right_bumper) {
                     claw.openServo();
//                     telemetry.addData("Claw State: ", "Open");
//                     telemetry.update();
                     sleep(350);
                     clawState = true;

                 }

                 //Open Claw
                 if(gamepad1.left_bumper) {
                     claw.closeServo();
//                     telemetry.addData("Claw State: ", "Closed");
//                     telemetry.update();
                     sleep(350);
                     clawState = false;
                 }

                 //Elevator Fully Up
                if (gamepad1.dpad_up){
                    elevator.setLiftPosition(4180);
                }

                //Elevator Down
                if (gamepad1.dpad_down){
                    elevator.setLiftPosition(50);
                }

                //Elevator Middle
                if (gamepad1.dpad_left){
                    elevator.setLiftPosition(1000); //3110
                }

            }
        }
    }
}
