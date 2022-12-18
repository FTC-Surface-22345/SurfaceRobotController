package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    private ElapsedTime runtime = new ElapsedTime();
    
    Servo leftServo;
    Servo rightServo;
    double servoPosition;
    double servoPosition2;

    public void init(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
    }

//    @Override
//    public void runOpMode() throws InterruptedException{
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//        servo = hardwareMap.servo.get("servo");
//        servo2 = hardwareMap.servo.get("servo2");
//        servoPosition = 0.55;
//        servoPosition2 = 1.0;
//        servo.setPosition(servoPosition);
//        servo2.setPosition(servoPosition2);
//        waitForStart();
//
//        while(opModeIsActive()) {
//            servoPosition = 0.75;
//            servoPosition2 = 0.8;
//            servo.setPosition(servoPosition);
//            servo2.setPosition(servoPosition2);
//        }
//
//    }
}