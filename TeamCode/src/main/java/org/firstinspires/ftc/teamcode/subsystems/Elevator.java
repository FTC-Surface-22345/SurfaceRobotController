//Property of FTC Team 22345 - All External users must request permission to access and utilize code
//Authors: Krish K. & Ronald Q.

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

//MINIMUM HEIGHT = 0, MAXIMUM HEIGHT = 4460;
public class Elevator {
    DcMotorEx left;
    DcMotorEx right;
    int height = 0;
    int stackNum = 5;
    int stackPos;
    double currentPosition;
    double leftPower;
    double rightPower;

    //INIT WITH testDrive HARDWARE MAP - @NOTE RENAME testDrive TO SurfaceHardwareMap
    public void init(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "left");
        left.setDirection(DcMotor.Direction.REVERSE);
        right = hardwareMap.get(DcMotorEx.class, "right");
        right.setDirection(DcMotor.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //DASHBOARD CONSTANTS INPUT
    public void moveLift(dashConstants.elevatorPos input) {
        switch (input) {
            case GROUND:
                move(50);
                break;

            case LOW:
                move(1780);
                break;

            case MIDDLE:
                move(2900);
                break;

            case HIGH:
                move(4180);
                break;

            case STACK:
                move(stackPos * stackNum);
                stackNum--;
                break;

            case GJUNC:
                move(150);
                break;
        }

    }

    //Constant change SetLiftPosition to tweak Values for Placing Cones on Junctions
    public void setLiftPosition(int target) {

        double currentPos = getPosition();

        if (currentPos < target) {
            //UP
            leftPower = -1;
            rightPower = 1;
        } else if (currentPos > target) {
            //DOWN
            leftPower = 0.45;
            rightPower = -0.45;
        }

        left.setTargetPosition(target);
        right.setTargetPosition(-target);

        left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        left.setPower(leftPower);
        right.setPower(rightPower);

        height = target;
        currentPosition = right.getCurrentPosition();
    }

    public void move(int target){
        setLiftPosition(target);
    }

    public double getPosition() {
        return ((right.getCurrentPosition() - left.getCurrentPosition())/2);
    }
}
