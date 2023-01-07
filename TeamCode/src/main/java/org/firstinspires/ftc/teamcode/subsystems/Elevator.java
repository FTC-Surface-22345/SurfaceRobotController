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

    public void init(HardwareMap hardwareMap) {
        leftElevator = hardwareMap.get(DcMotorEx.class, "Left Elevator");
        rightElevator = hardwareMap.get(DcMotorEx.class, "Right Elevator");
    }

    //target - current = error
    //error * gain (0.01) = newMotorPosition

    public void updateElevator() {
        currentPosition = leftElevator.getCurrentPosition();
        error = targetPosition - currentPosition;
        newMotorPower = error * gain;

        leftElevator.setPower(newMotorPower);
        rightElevator.setPower(newMotorPower);
    }

    public void setTargetPosition(double tempPosition){
        targetPosition = tempPosition;
    }




}
