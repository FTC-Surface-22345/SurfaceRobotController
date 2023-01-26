package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.PID;
import org.firstinspires.ftc.teamcode.subsystems.dashConstants;

public class Drive{

    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;

    double TPI = 53.76; //Ticks per Inch

    private BNO055IMU imu = null;

    //PIDs FOR MOVEMENT
    PID turnPID = new PID();

    PID FLPID = new PID();
    PID FRPID = new PID();
    PID BLPID = new PID();
    PID BRPID = new PID();

    //INITIALIZATION OF ALL MOTORS
    public void init(HardwareMap map){
        FL = map.get(DcMotorEx.class, "Front Left");
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR = map.get(DcMotorEx.class, "Front Right");
        FR.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BL = map.get(DcMotorEx.class, "Back Left");
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR = map.get(DcMotorEx.class, "Back Right");
        BR.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turnPID.init(new ElapsedTime(), dashConstants.turn[0], dashConstants.turn[1], dashConstants.turn[2]);

        FLPID.init(new ElapsedTime(), dashConstants.forward[0], dashConstants.forward[1], dashConstants.forward[2]);
        FRPID.init(new ElapsedTime(), dashConstants.forward[0], dashConstants.forward[1], dashConstants.forward[2]);
        BLPID.init(new ElapsedTime(), dashConstants.forward[0], dashConstants.forward[1], dashConstants.forward[2]);
        BRPID.init(new ElapsedTime(), dashConstants.forward[0], dashConstants.forward[1], dashConstants.forward[2]);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = map.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);
    }

    public void forward(double target){
        FLPID.setTarget(target);
        FL.setPower(FLPID.update((int)FL.getVelocity()));
        FRPID.setTarget(target);
        FR.setPower(FLPID.update((int)FL.getVelocity()));
        BLPID.setTarget(target);
        BL.setPower(FLPID.update((int)FL.getVelocity()));
        BRPID.setTarget(target);
        BR.setPower(FLPID.update((int)FL.getVelocity()));
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
