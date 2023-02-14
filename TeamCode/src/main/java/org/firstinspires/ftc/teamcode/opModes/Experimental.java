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
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Constants;

//SUBSYSTEMS
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Kamera;

@Autonomous(name = "Experimental")
public class Experimental extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    Constants.autoActions currentTrajectory = Constants.autoActions.IDLE;

    SampleMecanumDrive drivetrain;
    Elevator elevator = new Elevator();
    Claw claw = new Claw();
    Kamera camera = new Kamera();

    boolean clawState;
    double elevatorPosition;

    Pose2d startPos = new Pose2d(36, -62, Math.toRadians(90));
    Pose2d coneStackPos;

    int stack = 5;
    int high = 0;
    int mid = 0;
    int low = 0;

    void nextTrajectory(Constants.autoActions a) {
        double time = runtime.seconds();
        currentTrajectory = a;
        telemetry.addData("Trajectory: ", currentTrajectory);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new SampleMecanumDrive(hardwareMap);
        drivetrain.setPoseEstimate(startPos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //TRAJECTORIES
        Trajectory TO_HIGH_JUNC = drivetrain.trajectoryBuilder(startPos)
                //MOVE TO FIRST HIGH JUNC (24, 0)
                //LIFT ELEVATOR TO HIGH POSITION
                .lineTo(new Vector2d(36, -12)) //40, 10
                .addDisplacementMarker(
                        3,
                        () -> {
                            elevator.moveLift(Constants.elevatorPos.HIGH);
                        })
                //.lineToLinearHeading(new Pose2d(-32, 0, Math.toRadians(0)))
                .build();

        Trajectory STRAFE_TO_HIGH_JUNC = drivetrain.trajectoryBuilder(TO_HIGH_JUNC.end())
                .strafeTo(new Vector2d(23, -12)) //32, 0
                .build();

        Trajectory SCORE = drivetrain.trajectoryBuilder(STRAFE_TO_HIGH_JUNC.end())
                .lineTo(new Vector2d(23, -6))
                .addTemporalMarker(
                        2,
                        () -> {
                            claw.moveClaw(Constants.clawState.OPEN);
                        }
                )
                .build();

        Trajectory BACK_TO_CONE_STACK_LINE = drivetrain.trajectoryBuilder(SCORE.end())
                .lineTo(new Vector2d(23, -12))
                .build();

        Trajectory MOVE_TO_CONE_STACK = drivetrain.trajectoryBuilder(BACK_TO_CONE_STACK_LINE.end())
                .lineToLinearHeading(new Pose2d(62, -10, Math.toRadians(0)))
                .addDisplacementMarker(
                        3,
                        () -> {
                            elevator.moveLift(Constants.elevatorPos.STACK);
                        }
                )
                .addSpatialMarker(
                        new Vector2d(62, -10),
                        () -> {
                            claw.moveClaw(Constants.clawState.CLOSE);
                        }
                )
                .addTemporalMarker(
                        5,
                        () -> {
                            elevator.moveLift(Constants.elevatorPos.LOW);
                        }
                )


                .build();

        Trajectory BACK_TO_HIGH_JUNC = drivetrain.trajectoryBuilder(MOVE_TO_CONE_STACK.end())
                .lineToLinearHeading(new Pose2d(23, -10, Math.toRadians(90)))
                .addDisplacementMarker(
                        3,
                        () -> {
                            elevator.moveLift(Constants.elevatorPos.HIGH);
                        }
                )
                .build();

        Trajectory MOVE_TO_CENTER_LINE = drivetrain.trajectoryBuilder(BACK_TO_HIGH_JUNC.end())
                .strafeTo(new Vector2d(36, -10))
                .build();

        Trajectory MOVE_TO_PARK_LINE = drivetrain.trajectoryBuilder((MOVE_TO_CENTER_LINE).end())
                .lineTo(new Vector2d(36, -36))
                .build();

        runtime.reset();

        camera.init(hardwareMap);
        claw.init(hardwareMap);
        elevator.init(hardwareMap);

        Vector2d parkZone = new Vector2d();
        int zone = -1;

        claw.moveClaw(Constants.clawState.CLOSE);
        elevator.moveLift(Constants.elevatorPos.GROUND);

        while (opModeInInit())
        {
            zone = camera.getPosition();
            telemetry.addData("Parking Zone: ", zone);
            telemetry.update();
        }

        waitForStart();

        if (zone == 1){
            parkZone = new Vector2d(12, -12);
        }
        else if (zone == 2){
            parkZone = new Vector2d(36, -12);
        }
        else if (zone == 3){
            parkZone = new Vector2d(60, -12);
        }
        else if (zone == -1) {
            parkZone = new Vector2d(12, -12);
        }

        Trajectory PARK = drivetrain.trajectoryBuilder(MOVE_TO_CENTER_LINE.end())
                .strafeTo(parkZone)
                .build();
        //Trajectory PARK = drivetrain.trajectoryBuilder()

        telemetry.addLine("AUTONOMOUS READY...");
        telemetry.update();

        currentTrajectory = Constants.autoActions.TO_HIGH_JUNC;

        while (opModeIsActive()) {
            //Order of Trajectory
            //TO_HIGH_JUNC,               //MOVE FORWARD TO HIGH_JUNC
            //STRAFE_TO_HIGH_JUNC,        //STRAFE IN LINE WITH HIGH_JUNC
            //SCORE,                      //MOVE FORWARD, OPEN CLAW
            //BACK_TO_STACK_LINE,         //MOVE BACKWARD TOWARDS IN LINE WITH STACK
            //TURN_TO_CONE_STACK,         //TURN 90 DEGREES CLOCKWISE
            //MOVE_TO_CONE_STACK,         //MOVE FORWARD TO STACK
            //BACK_TO_HIGH_JUNC,          //MOVE BACKWARD WHILE TURNING 90 DEGREES COUNTERCLOCKWISE FACING HIGH JUNC
            //TO_HIGH_                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          JUNC_AFTER_STACK,    //MOVE FORWARD TO HIGH_JUNC, TURN 90 DEGREES COUNTER CLOCKWISE FACING HIGH_JUNC
            //MOVE_TO_PARK_LINE,          //MOVE BACKWARDS IN LINE TO STRAFE TO PARK
            //PARK,                       //MOVE TO PARK POSITION
            switch (currentTrajectory) {
                case TO_HIGH_JUNC:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(TO_HIGH_JUNC);
                        nextTrajectory(currentTrajectory.STRAFE_TO_HIGH_JUNC);
                    }
                    break;
                case STRAFE_TO_HIGH_JUNC:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(STRAFE_TO_HIGH_JUNC);
                        nextTrajectory(currentTrajectory.SCORE);
                    }
                case SCORE:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(SCORE);
                        high++;
                        if (high == 3) {
                            nextTrajectory(currentTrajectory.MOVE_TO_CENTER_LINE);
                        }
                        else {
                            nextTrajectory(currentTrajectory.MOVE_TO_CONE_STACK);
                        }
                    }
                    break;
                case BACK_TO_CONE_STACK_LINE:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(BACK_TO_CONE_STACK_LINE);
                        nextTrajectory(currentTrajectory.MOVE_TO_CONE_STACK);
                    }
                    break;
                case MOVE_TO_CONE_STACK:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(MOVE_TO_CONE_STACK);
                        nextTrajectory(currentTrajectory.BACK_TO_HIGH_JUNC);
                    }
                    break;
                case BACK_TO_HIGH_JUNC:
                    if (!drivetrain.isBusy()) {
                            drivetrain.followTrajectory(BACK_TO_HIGH_JUNC);
                            nextTrajectory(currentTrajectory.SCORE);
                    }
                    break;
                case MOVE_TO_CENTER_LINE:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(MOVE_TO_CENTER_LINE);
                        nextTrajectory(currentTrajectory.MOVE_TO_PARK_LINE);
                    }
                    break;
                case MOVE_TO_PARK_LINE:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(MOVE_TO_PARK_LINE);
                        nextTrajectory(currentTrajectory.PARK);
                    }
                case PARK:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(PARK);
                    }
                    break;
            }

        }
    }

}
