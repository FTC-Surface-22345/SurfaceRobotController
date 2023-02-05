package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utility.PID;

public class Drive {

    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;

    double forwardGain = .0008;
    double strafeGain = 0.018;

    double speed = 0.6;

    double TPI = dashConstants.COUNTS_PER_INCH; //Ticks per Inch

    //IMU & IMU VARIABLES
    BNO055IMU imu;
    Orientation angles;
    float IMUheading;
    float offsetAngle;

    //PIDs FOR MOVEMENT
    PID turnPID = new PID();
    PID FLPID = new PID();
    PID FRPID = new PID();
    PID BLPID = new PID();
    PID BRPID = new PID();

    double leftPower;
    double rightPower;

    MultipleTelemetry telemetry;

    //INITIALIZATION OF ALL MOTORS
    public void init(HardwareMap map, MultipleTelemetry tele) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        FL = map.get(DcMotorEx.class, "frontLeft");
        FL.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FR = map.get(DcMotorEx.class, "frontRight");
        //FR.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BL = map.get(DcMotorEx.class, "backLeft");
        BL.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BR = map.get(DcMotorEx.class, "backRight");
        //BR.setDirection(DcMotorSimple.Direction.REVERSE); // Delete if this breaks - only for conformity for now - In Autonomous and TeleOp
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        turnPID.init(new ElapsedTime(), dashConstants.turnPID[0], dashConstants.turnPID[1], dashConstants.turnPID[2]);

        //INITIALIZATION OF PIDS FOR EACH MOTOR
        FLPID.init(new ElapsedTime(), dashConstants.movePID[0], dashConstants.movePID[1], dashConstants.movePID[2]);
        FRPID.init(new ElapsedTime(), dashConstants.movePID[0], dashConstants.movePID[1], dashConstants.movePID[2]);
        BLPID.init(new ElapsedTime(), dashConstants.movePID[0], dashConstants.movePID[1], dashConstants.movePID[2]);
        BRPID.init(new ElapsedTime(), dashConstants.movePID[0], dashConstants.movePID[1], dashConstants.movePID[2]);

        telemetry = tele;


    }

    public void forward(double target) {
        while (FR.getCurrentPosition() < target) {
            updateIMU();

            FL.setPower(speed);
            BL.setPower(speed);
            FR.setPower(speed);
            BR.setPower(speed);

            leftPower = speed;
            rightPower = speed;

            while (FR.getCurrentPosition() < target && Math.abs(IMUheading) > 0.1) {
                updateIMU();
                leftPower = leftPower + (0 - IMUheading) * -forwardGain;
                rightPower = rightPower + (0 - IMUheading) * forwardGain;
                FL.setPower(leftPower);
                BL.setPower(leftPower);
                FR.setPower(rightPower);
                BR.setPower(rightPower);

                telemetry.addData("Left Power: ", leftPower);
                telemetry.addData("Right Power: ", rightPower);

                telemetry.addData("FL: ", FL.getCurrentPosition());
                telemetry.addData("FR: ", FR.getCurrentPosition());
                telemetry.addData("BL: ", BL.getCurrentPosition());
                telemetry.addData("BR: ", BR.getCurrentPosition());
                telemetry.update();
            }
        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        leftPower = 0;
        rightPower = 0;

        telemetry.addData("FL: ", FL.getCurrentPosition());
        telemetry.addData("FR: ", FR.getCurrentPosition());
        telemetry.addData("BL: ", BL.getCurrentPosition());
        telemetry.addData("BR: ", BR.getCurrentPosition());
        telemetry.update();
    }

    public void backward(double target) {
        while (FR.getCurrentPosition() < target) {
            FL.setDirection(DcMotorSimple.Direction.FORWARD);
            FR.setDirection(DcMotorSimple.Direction.REVERSE);
            BL.setDirection(DcMotorSimple.Direction.FORWARD);
            BR.setDirection(DcMotorSimple.Direction.REVERSE);

            updateIMU();

            FL.setPower(speed);
            BL.setPower(speed);
            FR.setPower(speed);
            BR.setPower(speed);

            leftPower = speed;
            rightPower = speed;

            while (FR.getCurrentPosition() < target && Math.abs(IMUheading) > 0.1) {
                updateIMU();
                leftPower = leftPower + (0 - IMUheading) * -forwardGain;
                rightPower = rightPower + (0 - IMUheading) * forwardGain;
                FL.setPower(leftPower);
                BL.setPower(leftPower);
                FR.setPower(rightPower);
                BR.setPower(rightPower);

                telemetry.addData("Left Power: ", leftPower);
                telemetry.addData("Right Power: ", rightPower);

                telemetry.addData("FL: ", FL.getCurrentPosition());
                telemetry.addData("FR: ", FR.getCurrentPosition());
                telemetry.addData("BL: ", BL.getCurrentPosition());
                telemetry.addData("BR: ", BR.getCurrentPosition());
                telemetry.update();
            }
        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        leftPower = 0;
        rightPower = 0;

        telemetry.addData("FL: ", FL.getCurrentPosition());
        telemetry.addData("FR: ", FR.getCurrentPosition());
        telemetry.addData("BL: ", BL.getCurrentPosition());
        telemetry.addData("BR: ", BR.getCurrentPosition());
        telemetry.update();

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void turn(double target) {


    }

    public void strafeLeft(double target) {
        while (FR.getCurrentPosition() < target) {
            FL.setDirection(DcMotorSimple.Direction.FORWARD);
            BR.setDirection(DcMotorSimple.Direction.REVERSE);

            updateIMU();

            FL.setPower(speed);
            BL.setPower(speed);
            FR.setPower(speed);
            BR.setPower(speed);

            leftPower = speed;
            rightPower = speed;

            updateIMU();

            leftPower = leftPower + (0 - IMUheading) * -strafeGain;
            rightPower = rightPower + (0 - IMUheading) * strafeGain;

            FL.setPower(leftPower);
            BL.setPower(leftPower);
            FR.setPower(rightPower);
            BR.setPower(rightPower);

            telemetry.addData("Left Power: ", leftPower);
            telemetry.addData("Right Power: ", rightPower);

            telemetry.addData("FL: ", FL.getCurrentPosition());
            telemetry.addData("FR: ", FR.getCurrentPosition());
            telemetry.addData("BL: ", BL.getCurrentPosition());
            telemetry.addData("BR: ", BR.getCurrentPosition());
            telemetry.update();

        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        leftPower = 0;
        rightPower = 0;

        telemetry.addData("FL: ", FL.getCurrentPosition());
        telemetry.addData("FR: ", FR.getCurrentPosition());
        telemetry.addData("BL: ", BL.getCurrentPosition());
        telemetry.addData("BR: ", BR.getCurrentPosition());
        telemetry.update();

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void strafeRight(double target) {
        while (FR.getCurrentPosition() < target) {
            BL.setDirection(DcMotorSimple.Direction.FORWARD);
            FR.setDirection(DcMotorSimple.Direction.REVERSE);

            updateIMU();

            FL.setPower(speed);
            BL.setPower(speed);
            FR.setPower(speed);
            BR.setPower(speed);

            leftPower = speed;
            rightPower = speed;


            updateIMU();

            leftPower = leftPower + (0 - IMUheading) * -strafeGain;
            rightPower = rightPower + (0 - IMUheading) * strafeGain;

            FL.setPower(leftPower);
            BL.setPower(leftPower);
            FR.setPower(rightPower);
            BR.setPower(rightPower);

            telemetry.addData("Left Power: ", leftPower);
            telemetry.addData("Right Power: ", rightPower);

            telemetry.addData("FL: ", FL.getCurrentPosition());
            telemetry.addData("FR: ", FR.getCurrentPosition());
            telemetry.addData("BL: ", BL.getCurrentPosition());
            telemetry.addData("BR: ", BR.getCurrentPosition());
            telemetry.update();


        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        leftPower = 0;
        rightPower = 0;

        telemetry.addData("FL: ", FL.getCurrentPosition());
        telemetry.addData("FR: ", FR.getCurrentPosition());
        telemetry.addData("BL: ", BL.getCurrentPosition());
        telemetry.addData("BR: ", BR.getCurrentPosition());
        telemetry.update();

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void reset() {
        FL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        leftPower = 0;
        rightPower = 0;
    }

    public void resetIMU() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        offsetAngle = angles.firstAngle;
    }

    public void updateIMU() {
        IMUheading = angles.firstAngle - offsetAngle;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData("roll", angles.secondAngle);
        telemetry.addData("pitch", angles.thirdAngle);
        telemetry.addData("IMUheading", IMUheading);
        telemetry.addData("offset", offsetAngle);
        telemetry.update();
    }


}
