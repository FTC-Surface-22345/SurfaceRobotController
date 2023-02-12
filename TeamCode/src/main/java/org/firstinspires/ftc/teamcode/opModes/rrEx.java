package org.firstinspires.ftc.teamcode.opModes;

//TELEMETRY & DASHBOARD
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

//ROADRUNNER
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

//UTIL
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Constants;

//SUBSYSTEMS
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Kamera;

@Autonomous(name = "RoadRunner Experimental")
@Config
public class rrEx extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    Constants.autoActions currentTrajectory = Constants.autoActions.IDLE;

    boolean clawState;
    int elevatorPosition;

    SampleMecanumDrive drivetrain;
    Elevator elevator = new Elevator();
    Claw claw = new Claw();
    Kamera camera = new Kamera();

    Pose2d startPos = new Pose2d(60, 36, Math.toRadians(90));
    Pose2d coneStackPos;

    int stack = 5;
    int high = 0;
    int mid = 0;
    int low = 0;

    void nextTrajectory(Constants.autoActions a){
        double time = runtime.seconds();
        currentTrajectory = a;
    }

    @Override
    public void runOpMode() {
        drivetrain = new SampleMecanumDrive(hardwareMap);
        drivetrain.setPoseEstimate(startPos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //TRAJECTORIES
        Trajectory TO_HIGHJUNC = drivetrain.trajectoryBuilder(startPos)
                //MOVE TO FIRST HIGH JUNC (24, 0)
                //LIFT ELEVATOR TO HIGH POSITION
                .lineTo(new Vector2d(0, 36))
                .addDisplacementMarker(
                        3,
                        () -> {
                            elevator.moveLift(Constants.elevatorPos.HIGH);
                        })
                .build();


    }


}
