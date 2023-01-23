package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config("FTCDashboardConstants")
public class dashConstants {
    public dashConstants() {} //Empty Constructor

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
        TO_BEACON,         //GO FORWARD INTO THE BEACON PUSHING IT OUT OF THE WAY
        TO_HIGHJUNC,       //STRAFE RIGHT, RAISE ELEVATOR TO HIGH JUNCTION CENTERED WITH SUBSTATION
        SCORE_PRELOAD,     //MOVE FORWARD, OPEN CLAW ABOVE JUNCTION, MOVE BACK, LOWER ELEVATOR
        TO_STACK_LEFT,     //STRAFE RIGHT TO CONE STACK, MOVE FORWARD, TURN LEFT,
        PICK_CONE_5,       //MOVE FORWARD, RAISE ELEVATOR TO CONE 5 HEIGHT, CLOSE CLAW, RAISE ELEVATOR UP TO RELEASE CONE FROM STACK, REVERSE
        TO_HIGHJUNC_AFTER, //TURN TILL FACING HIGH JUNCTION NEAR TERMINALS, RAISE ELEVATOR, MOVE FORWARD,
        SCORE_CONE_5,      //OPEN CLAW, REVERSE, LOWER ELEVATOR,
        TO_SUBSTATION,     //TURN TILL PERPENDICULAR WITH CORRESPONDING TEAM AREA, REVERSE INTO TEAM WALL CORRESPONDING WITH TEAM AREA, STRAFE RIGHT OR LEFT DEPENDING ON LOCATION OF SUBSTATION
        TO_TERMINAL,       //TURN TILL PERPENDICULAR WITH CORRESPONDING TEAM AREA, REVERSE INTO TEAM WALL CORRESPONDING WITH TEAM AREA, STRAFE RIGHT OR LEFT DEPENDING ON LOCATION OF TERMINAL
        PARK,              //TURN TILL PERPENDICULAR WITH CORRESPONDING TEAM AREA, REVERSE INTO ZONES, STRAFE RIGHT OR LEFT DEPENDING ON ZONE
        IDLE               //STOP ALL TASKS AND WAIT FOR FURTHER INSTRUCTION

    }

    //FIXED CONSTANTS FOR CLAW SERVO POSITIONS - DO NOT CHANGE UNLESS ERRORS WITH CLAW
    public static double openClawL = .55;
    public static double openClawR = .80;
    public static double closeClawL = 1;
    public static double closeClawR = .75;

    public static int C1;
    public static int C2;
    public static int C3;
    public static int C4;
    public static int C5;

}

