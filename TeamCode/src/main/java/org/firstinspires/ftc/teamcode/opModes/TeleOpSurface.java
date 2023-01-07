//Property of FTC Team 22345 - All External users must request permission to access and utilize code
//Authors: Krish K. & Ronald Q.

package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

@TeleOp(name = "TeleOpSurface")
public class TeleOpSurface extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private Claw mainClaw = new Claw();


    @Override
    public void runOpMode() {
        //Front Drive Motors IInitialization
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        //Back Drive Motors Initialization
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //Claw Initialization
        mainClaw.init(hardwareMap);
        boolean clawState = false; //closed

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Robot Status", "Initialized");
                telemetry.update();

                backLeft.setPower((gamepad1.left_stick_y + (gamepad1.left_stick_x - gamepad1.right_stick_x)) / 2);
                frontLeft.setPower((gamepad1.left_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x)) / 2);
                backRight.setPower((gamepad1.left_stick_y - (gamepad1.left_stick_x - gamepad1.right_stick_x)) / 2);
                frontRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / 2);
                if (gamepad1.right_bumper) {
                    if (clawState == false) {
                        mainClaw.openServo();
                        telemetry.addData("Claw: ", "Open");
                        telemetry.update();
                        clawState = true;
                        break;
                    }

                    if (clawState == true) {
                        mainClaw.closeServo();
                        telemetry.update();
                        telemetry.addData("Claw: ", "Closed");
                        clawState = false;
                        break;
                    }

//                    while(gamepad1.right_bumper){
//                        int state = 0;
//                        switch (state){
//                            case 0:
//                                mainClaw.openServo();
//                                telemetry.addData("Claw: ", "Open");
//                                telemetry.update();
//                                clawState = true;
//                                state = 1;
//                                break;
//                            case 1:
//                                mainClaw.closeServo();
//                                telemetry.update();
//                                telemetry.addData("Claw: ", "Closed");
//                                clawState = false;
//                                state = 0;
//                                break;
//                        }
//
//                    }


                }


            }
        }
    }
}
