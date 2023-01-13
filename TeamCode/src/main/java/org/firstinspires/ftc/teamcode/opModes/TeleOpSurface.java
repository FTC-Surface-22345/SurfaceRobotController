//Property of FTC Team 22345 - All External users must request permission to access and utilize code
//Authors: Krish K. & Ronald Q.

package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

@TeleOp(name = "TeleOpSurface")
public class TeleOpSurface extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;

    private Claw claw = new Claw();

    private Elevator elevator = new Elevator();


    @Override
    public void runOpMode() {
        //Front Drive Motors Initialization
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        //Back Drive Motors Initialization
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //Claw Initialization
        //claw.init(hardwareMap);
        //boolean clawState = false; //closed

        //Elevator Initialization
        elevator.init(hardwareMap);
        int elevatorLevel = 0;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Robot Status", "Initialized");
                telemetry.update();

                backLeft.setPower((gamepad1.left_stick_y + (gamepad1.left_stick_x - gamepad1.right_stick_x)) / 2);
                frontLeft.setPower((gamepad1.left_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x)) / 2);
                backRight.setPower((gamepad1.left_stick_y - (gamepad1.left_stick_x - gamepad1.right_stick_x)) / 2);
                frontRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / 2);
                // if (gamepad1.right_bumper) {
                //     if (clawState == false) {
                //         claw.openServo();
                //         telemetry.addData("Claw State: ", "Open");
                //         telemetry.update();
                //         sleep(250);
                //         clawState = true;
                //     }

                //     if (clawState == true) {
                //         claw.closeServo();
                //         telemetry.update();
                //         telemetry.addData("Claw State: ", "Closed");
                //         sleep(250);
                //         clawState = false;
                //     }
                // }

                if (gamepad1.dpad_up){
                    elevator.setLiftPosition(4440);
                }
                
                if (gamepad1.dpad_down){
                    elevator.setLiftPosition(0);
                }

            }
        }
    }
}
