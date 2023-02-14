package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config("FTCDashboardConstants")
public class Constants {
    public Constants() {} //Empty Constructor

    //Elevator Constant Values: GROUND - 50, LOW - 1780, MIDDLE - 3110, HIGH - 4180, STACK - volatile value, GJUNC - 150 <-- @NOTE VALUE MAY BE INACCURATE, NEEDS TESTING
    public enum elevatorPos{
        GROUND,
        LOW,
        MIDDLE,
        HIGH,
        STACK,
        GJUNC
    }

    //Claw Constant Values: OPEN - 1, CLOSE - 0
    public enum clawState{
        OPEN,
        CLOSE
    }

    //AUTONOMOUS CONSTANTS
    public enum autoActions{
        TO_HIGH_JUNC,                //MOVE FORWARD TO HIGH_JUNC
        STRAFE_TO_HIGH_JUNC,        //STRAFE IN LINE WITH HIGH_JUNC
        SCORE,                       //MOVE FORWARD, OPEN CLAW
        BACK_TO_CONE_STACK_LINE,    //MOVE BACKWARD TOWARDS IN LINE WITH STACK
        MOVE_TO_CONE_STACK,         //MOVE FORWARD TO STACK
        PICK_FROM_CONE_STACK,       //MOVE LIFT UPWARDS AFTER CLAW CLOSES
        BACK_TO_HIGH_JUNC,          //MOVE BACKWARD WHILE TURNING 90 DEGREES COUNTERCLOCKWISE FACING HIGH JUNC
        TO_HIGHJUNC_AFTER_STACK,    //MOVE FORWARD TO HIGH_JUNC, TURN 90 DEGREES COUNTER CLOCKWISE FACING HIGH_JUNC
        MOVE_TO_CENTER_LINE,        //STRAFE TO CENTER LINE
        MOVE_TO_PARK_LINE,          //MOVE BACKWARDS IN LINE TO STRAFE TO PARK
        PARK,                       //MOVE TO PARK POSITION
        IDLE                        //STOP ALL TASKS AND WAIT FOR FURTHER INSTRUCTION

    }

    //FIXED CONSTANTS FOR CLAW SERVO POSITIONS - DO NOT CHANGE UNLESS ERRORS WITH CLAW
    public static double openClawL = .6;
    public static double openClawR = 1.1;
    public static double closeClawL = 0.80;
    public static double closeClawR = 0.80;

    public static int C1;
    public static int C2;
    public static int C3;
    public static int C4;
    public static int C5;

    public static double P = 0.01;
    public static double I = 0;
    public static double D = 0;

    public static double turnP = 0.06;
    public static double turnI = 0;
    public static double turnD = 0;

    public static double[] movePID = {P, I, D};
    public static double[] turnPID = {turnP, turnI, turnD};

    public static double targetHeading = 0;
    public static double driveSpeed = 0;
    public static double turnSpeed = 0;
    public static double leftSpeed = 0;
    public static double rightSpeed = 0;
    public static double leftTarget = 0;
    public static double rightTarget = 0;

    public static double HEADING_THRESHOLD = 0.08;

    public static double COUNTS_PER_MOTOR_REV = 537.7 ; // GoBILDA 312 RPM Yellow Jacket
    public static double DRIVE_GEAR_REDUCTION = 1.0 ; // No External Gearing.
    public static double WHEEL_DIAMETER_INCHES = 4.0 ; // For figuring circumference
    public static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


}

