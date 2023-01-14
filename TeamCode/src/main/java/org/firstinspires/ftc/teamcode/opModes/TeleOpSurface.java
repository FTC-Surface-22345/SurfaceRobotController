//Property of FTC Team 22345 - All External users must request permission to access and utilize code
//Authors: Krish K. & Ronald Q.

package org.firstinspires.ftc.teamcode.opModes;

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
        //Front Drive Motors Initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp

        //Back Drive Motors Initialization
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
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
                telemetry.update();

                backLeft.setPower((gamepad1.left_stick_y + (gamepad1.left_stick_x - gamepad1.right_stick_x)) / 2);
                frontLeft.setPower((gamepad1.left_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x)) / 2);
                backRight.setPower((gamepad1.left_stick_y - (gamepad1.left_stick_x - gamepad1.right_stick_x)) / 2);
                frontRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / 2);
                 if (gamepad1.right_bumper) {
                     if (!clawState) {
                         claw.openServo();
                         telemetry.addData("Claw State: ", "Open");
                         telemetry.update();
                         sleep(350);
                         clawState = true;
                     }

                     if (clawState) {
                         claw.closeServo();
                         telemetry.update();
                         telemetry.addData("Claw State: ", "Closed");
                         sleep(350);
                         clawState = false;
                     }
                 }

                if (gamepad1.dpad_up){
                    elevator.setLiftPosition(4230);
                }
                
                if (gamepad1.dpad_down){
                    elevator.setLiftPosition(50);
                }

            }
        }
    }
}
