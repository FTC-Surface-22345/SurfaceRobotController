//Property of FTC Team 22345 - All External users must request permission to access and utilize code
//Authors: Krish K. & Ronald Q.

package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

@Autonomous
public class AutoOpSurface extends LinearOpMode{

    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx frontRight;

    private Claw mainClaw = new Claw();


    @Override
    public void runOpMode(){
        //Front Drive Motors Initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        //Back Drive Motors Initialization
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        //Claw Initialization
        mainClaw.init(hardwareMap);
        boolean clawState = false; //closed

        waitForStart();
        if (opModeIsActive())
        {
            //In Progress...
        }

    }

}
