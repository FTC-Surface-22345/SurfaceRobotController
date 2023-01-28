package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utility.PID;

public class Drive {

    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;

    double TPI = dashConstants.COUNTS_PER_INCH; //Ticks per Inch

    BNO055IMU imu;
    Orientation angle;


    YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;
    //PIDs FOR MOVEMENT
    PID turnPID = new PID();
    PID FLPID = new PID();
    PID FRPID = new PID();
    PID BLPID = new PID();
    PID BRPID = new PID();

    private double heading;
    private double headingOffset = 0;
    private final double headingError = 0;

    //INITIALIZATION OF ALL MOTORS
    public void init(HardwareMap map) {
        imu = map.get(BNO055IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        FL = map.get(DcMotorEx.class, "frontLeft");
        FL.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FR = map.get(DcMotorEx.class, "frontRight");
        //FR.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        BL = map.get(DcMotorEx.class, "backLeft");
        BL.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        BR = map.get(DcMotorEx.class, "backRight");
        //BR.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turnPID.init(new ElapsedTime(), dashConstants.turn[0], dashConstants.turn[1], dashConstants.turn[2]);

        FLPID.init(new ElapsedTime(), dashConstants.forward[0], dashConstants.forward[1], dashConstants.forward[2]);
        FRPID.init(new ElapsedTime(), dashConstants.forward[0], dashConstants.forward[1], dashConstants.forward[2]);
        BLPID.init(new ElapsedTime(), dashConstants.forward[0], dashConstants.forward[1], dashConstants.forward[2]);
        BRPID.init(new ElapsedTime(), dashConstants.forward[0], dashConstants.forward[1], dashConstants.forward[2]);

        resetHeading();
    }

    public void forward(double target) {
        FLPID.setTarget(target);
        FRPID.setTarget(target);
        BLPID.setTarget(target);
        BRPID.setTarget(target);

    }

    public void turn() {


    }

    public void strafe() {

    }

    public void update() {

        FL.setPower(FLPID.update(FL.getCurrentPosition()));

        FR.setPower(FLPID.update(FR.getCurrentPosition()));

        BL.setPower(FLPID.update(BL.getCurrentPosition()));

        BR.setPower(FLPID.update(BR.getCurrentPosition()));
    }

    public double getFLPos() {
        return FL.getCurrentPosition();
    }

    public double getFRPos() {
        return FR.getCurrentPosition();
    }

    public double getBLPos() {
        return BL.getCurrentPosition();
    }

    public double getBRPos() {
        return BR.getCurrentPosition();
    }

    public void reset() {
        FL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void IMUreading() {
        zYaw();
        xPitch();
        yRoll();

        YawVel();
        PitchVel();
        RollVel();
    }

    public double zYaw() {
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public double xPitch() {
        return orientation.getPitch(AngleUnit.DEGREES);
    }

    public double yRoll() {
        return orientation.getRoll(AngleUnit.DEGREES);
    }

    public double YawVel() {
        return angularVelocity.zRotationRate;
    }

    public double PitchVel() {
        return angularVelocity.xRotationRate;
    }

    public double RollVel() {
        return angularVelocity.yRotationRate;
    }

    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        heading = 0;
    }

}
