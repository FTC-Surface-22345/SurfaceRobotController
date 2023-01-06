//Krish
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
    double updatePosition;

    public void init(HardwareMap hardwareMap){
        leftElevator = hardwareMap.get(DcMotorEx.class, "Left Elevator");
        rightElevator = hardwareMap.get(DcMotorEx.class, "Right Elevator");
    }



}
