package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.PID;

public class Drive{

    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;

    double TPI = 53.76; //Ticks per Inch

    private BNO055IMU IMU = null;

    //PIDs FOR MOVEMENT
    PID turnPID = new PID();

    PID FLPID = new PID();
    PID FRPID = new PID();
    PID BLPID = new PID();
    PID BRPID = new PID();

    //INITIALIZATION OF ALL MOTORS
    public void init(HardwareMap map){
        FL = map.get(DcMotorEx.class, "Front Left");

        FR = map.get(DcMotorEx.class, "Front Right");

        BL = map.get(DcMotorEx.class, "Back Left");

        BR = map.get(DcMotorEx.class, "Back Right");

        turnPID.init(new ElapsedTime(), 0, 0, 0);

        FLPID.init(new ElapsedTime(), 0, 0, 0);
        FRPID.init(new ElapsedTime(), 0, 0, 0);
        BLPID.init(new ElapsedTime(), 0, 0, 0);
        BRPID.init(new ElapsedTime(), 0, 0, 0);
    }

    public void forward(double target){
        FLPID.setTarget(target);
        FRPID.setTarget(target);
        BLPID.setTarget(target);
        BRPID.setTarget(target);
    }

    public void turn(){

    }

    public void strafe(){

    }

    public void update(){

    }

    public double getPos(){
        return FR.getCurrentPosition();
    }


}
