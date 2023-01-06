//Ronald
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw{
    private ElapsedTime runtime = new ElapsedTime();

    Servo leftServo;
    Servo rightServo;
    double openPositionLeft = 0.55; //While opMode Active --> 0.75
    double closePositionLeft = 0.75;
    double openPositionRight = 1.0; //While opMode Active --> 0.8
    double closePositionRight = 0.8;

    public void init(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
    }

    public void openServo(){
        leftServo.setPosition(openPositionLeft);
        rightServo.setPosition(openPositionRight);
    }

    public void closeServo(){
        leftServo.setPosition(closePositionLeft);
        rightServo.setPosition(closePositionRight);
    }



}