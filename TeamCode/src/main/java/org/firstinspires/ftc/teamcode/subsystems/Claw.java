//Property of FTC Team 22345 - All External users must request permission to access and utilize code
//Authors: Ronald Q. & Krish K.

package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.closeClawL;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.closeClawR;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.openClawL;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.openClawR;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw{
    private ElapsedTime runtime = new ElapsedTime();

    Servo leftServo;
    Servo rightServo;

    public boolean clawState;
//    double openPositionLeft = 0.55; //While opMode Active --> 0.75
//    double closePositionLeft = 0.80;
//    double openPositionRight = 1.0; //While opMode Active --> 0.8
//    double closePositionRight = 0.75;

    public void init(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(Servo.class, "leftClaw");
        rightServo = hardwareMap.get(Servo.class, "rightClaw");
    }

    public void open(){
        leftServo.setPosition(openClawL);
        rightServo.setPosition(openClawR);
    }

    public void close(){
        leftServo.setPosition(closeClawL);
        rightServo.setPosition(closeClawR);
    }


    public void moveClaw(Constants.clawState input){
        switch (input){
            case OPEN:
                open();
                clawState = false;
                break;

            case CLOSE:
                close();
                clawState = true;
                break;

        }
    }

    public boolean isBusy(){
        if (leftServo.getPosition() <= 0.7 || leftServo.getPosition() >= 0.77) {
            return true;
        }
        return false;
    }


}