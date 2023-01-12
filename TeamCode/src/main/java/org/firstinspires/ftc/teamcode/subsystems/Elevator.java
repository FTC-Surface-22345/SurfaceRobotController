//Property of FTC Team 22345 - All External users must request permission to access and utilize code
//Authors: Krish K. & Ronald Q.

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator{
    DcMotorEx leftElevator;
    DcMotorEx rightElevator;
    double setPosition;
    double retractedPosition = 0;
    double targetPosition;
    double currentPosition;
    double error;
    double gain = 0.01;
    double newMotorPower;
    double leftPower;
    double rightPower;

    public void init(HardwareMap hardwareMap) { //Complete
        leftElevator = hardwareMap.get(DcMotorEx.class, "LeftElevator");
        rightElevator = hardwareMap.get(DcMotorEx.class, "RightElevator");

        leftElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);`
    }


    public void setLiftPosition (double targetPos){

        double currentPos = getPosition();

        if (currentPos < targetPos) {
            // Going up
            leftPower = 1;
            rightPower = -1;
        } else if (currentPos > targetPos) {
            // Going down
            leftPower = -0.5;
            rightPower = 0.5;
        }

        left.setTargetPosition((int) targetPos);
        right.setTargetPosition((int) -targetPos);

        left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        left.setPower(leftPower);
        right.setPower(rightPower);
        
    }

    public double getPosition(){
        return leftElevator.getCurrentPosition();
    }




}
