//Property of FTC Team 22345 - All External users must request permission to access and utilize code
//Authors: Krish K. & Ronald Q.

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator{
    DcMotorEx leftElevator;
    DcMotorEx rightElevator;
    double setPosition;
    public int minHeight = 0;
    public int maxHeight = 4460;
    double targetPosition;
    double currentPosition;
    double error;
    double gain = 0.01;
    double newMotorPower;
    double leftPower;
    double rightPower;

    public void init(HardwareMap hardwareMap) { //Complete
        leftElevator = hardwareMap.get(DcMotorEx.class, "LeftElevator");
        leftElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        rightElevator = hardwareMap.get(DcMotorEx.class, "RightElevator");
        rightElevator.setDirection(DcMotorSimple.Direction.REVERSE);

        leftElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void setLiftPosition (int targetPos){

        double currentPos = getPosition();

        if (currentPos < targetPos) {
            // Going up
            leftPower = -1;
            rightPower = 1;
        } else if (currentPos > targetPos) {
            // Going down
            leftPower = 0.8;
            rightPower = -0.8;
        }

        leftElevator.setTargetPosition((int) targetPos);
        rightElevator.setTargetPosition((int) -targetPos);

        leftElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftElevator.setPower(leftPower);
        rightElevator.setPower(rightPower);
        
    }

    public double getPosition(){
        return rightElevator.getCurrentPosition();
    }




}
