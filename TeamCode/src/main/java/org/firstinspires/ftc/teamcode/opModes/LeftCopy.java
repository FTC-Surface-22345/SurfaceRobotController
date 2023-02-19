package org.firstinspires.ftc.teamcode.opModes;

//TELEMETRY & DASHBOARD

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Kamera;

@Autonomous(name = "Autonomous Left No WEBCAM")
public class LeftCopy extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    Constants.autoActions currentTrajectory = Constants.autoActions.IDLE;

    SampleMecanumDrive drivetrain;
    Elevator elevator = new Elevator();
    Claw claw = new Claw();
    Kamera camera = new Kamera();

    Pose2d startPos = new Pose2d(-39.5, -59.6, Math.toRadians(90));

    int stack = 0;
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
        Trajectory READY = drivetrain.trajectoryBuilder(startPos)
                .lineTo(new Vector2d(-34, -50))
                .build();

        Trajectory TO_HIGH_JUNC = drivetrain.trajectoryBuilder(READY.end())
                .lineTo(new Vector2d(-36, -12)) //40, 10
                .addDisplacementMarker(
                        3,
                        () -> {
                            elevator.moveLift(Constants.elevatorPos.HIGH);
                        })
                .build();

        Trajectory STRAFE_TO_HIGH_JUNC = drivetrain.trajectoryBuilder(TO_HIGH_JUNC.end())
                .strafeTo(new Vector2d(-23.6, -12)) //32, 0
                .build();

        Trajectory SCORE = drivetrain.trajectoryBuilder(STRAFE_TO_HIGH_JUNC.end())
                .lineTo(new Vector2d(-23.6, -4.5))
                .addTemporalMarker(
                        2,
                        () -> {
                            claw.moveClaw(Constants.clawState.OPEN);
                        }
                )
                .build();

        Trajectory BACK_TO_CONE_STACK_LINE = drivetrain.trajectoryBuilder(SCORE.end())
                .lineTo(new Vector2d(-23.6, -10.6))
                .build();

        Trajectory MOVE_TO_CENTER_LINE = drivetrain.trajectoryBuilder(BACK_TO_CONE_STACK_LINE.end())
                .strafeTo(new Vector2d(-36, -10.6))
                .addDisplacementMarker(
                        1,
                        () -> {
                            elevator.moveLift(Constants.elevatorPos.GROUND);
                        }
                )
                .build();

        Trajectory MOVE_TO_CONE_STACK = drivetrain.trajectoryBuilder(MOVE_TO_CENTER_LINE.end())
                .lineToLinearHeading(new Pose2d(-62, -10.6, Math.toRadians(180)))
                .addDisplacementMarker(
                        3,
                        () -> {
                            elevator.moveLift(Constants.elevatorPos.STACK);
                        }
                )
                .addSpatialMarker(
                        new Vector2d(-62, -10.6),
                        () -> {
                            claw.moveClaw(Constants.clawState.CLOSE);
                        }
                )
                .build();

        Trajectory BACK_TO_HIGH_JUNC = drivetrain.trajectoryBuilder(MOVE_TO_CONE_STACK.end())
                .lineToLinearHeading(new Pose2d(-23.6, -10.6, Math.toRadians(90)))
                .addDisplacementMarker(
                        3,
                        () -> {
                            elevator.moveLift(Constants.elevatorPos.HIGH);
                        }
                )
                .build();



        Trajectory MOVE_TO_PARK_LINE = drivetrain.trajectoryBuilder((MOVE_TO_CENTER_LINE).end())
                .lineTo(new Vector2d(-36, -36))
                .build();

        runtime.reset();

        camera.init(hardwareMap);
        claw.init(hardwareMap);
        elevator.init(hardwareMap);

        Pose2d parkZone = new Pose2d();
        int zone = -1;

        claw.moveClaw(Constants.clawState.CLOSE);

        while (opModeInInit()) {
//            zone = camera.getPosition();
//            telemetry.addData("Parking Zone: ", zone);
//            telemetry.update();
        }

        waitForStart();

//        camera.webcam.stopStreaming();
//        camera.webcam.stopRecordingPipeline();

        elevator.moveLift(Constants.elevatorPos.GROUND);

//        if (zone == 1) {
//            parkZone = new Pose2d(-60, -11.3, Math.toRadians(90));
//        } else if (zone == 2) {
//            parkZone = new Pose2d(-36, -11.3, Math.toRadians(90));
//        } else if (zone == 3) {
//            parkZone = new Pose2d(-12, -11.3, Math.toRadians(90));
//        } else if (zone == -1) {
//            parkZone = new Pose2d(-12, -11.3, Math.toRadians(90));
//        }

//        Trajectory PARK = drivetrain.trajectoryBuilder(BACK_TO_CONE_STACK_LINE.end())
//                .lineToLinearHeading(parkZone)
//                .build();
        //Trajectory PARK = drivetrain.trajectoryBuilder()

        telemetry.addLine("AUTONOMOUS READY...");
        telemetry.update();

        currentTrajectory = Constants.autoActions.READY;

        while (opModeIsActive()) {
            telemetry.addData("High Scored: ", high);
            telemetry.addData("Cone Stack: ", stack);
            telemetry.addData("Current Trajectory: ", currentTrajectory);
            telemetry.update();

            switch (currentTrajectory) {
                case READY:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(READY);
                        nextTrajectory(Constants.autoActions.TO_HIGH_JUNC);
                    }
                case TO_HIGH_JUNC:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(TO_HIGH_JUNC);
                        nextTrajectory(Constants.autoActions.STRAFE_TO_HIGH_JUNC);
                    }
                    break;
                case STRAFE_TO_HIGH_JUNC:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(STRAFE_TO_HIGH_JUNC);
                        nextTrajectory(Constants.autoActions.SCORE);
                    }
                case SCORE:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(SCORE);
                        high++;
                        nextTrajectory(Constants.autoActions.BACK_TO_CONE_STACK_LINE);
                    }
                    break;
                case BACK_TO_CONE_STACK_LINE:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(BACK_TO_CONE_STACK_LINE);
                        if (high == 3) {
                            nextTrajectory(Constants.autoActions.PARK);
                            break;
                        }
                        nextTrajectory(Constants.autoActions.MOVE_TO_CENTER_LINE); //MOVE_TO_CONE_STACK
                    }
                    break;
                case MOVE_TO_CENTER_LINE:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(MOVE_TO_CENTER_LINE);
                        nextTrajectory(Constants.autoActions.MOVE_TO_CONE_STACK);
                    }
                    break;
                case MOVE_TO_CONE_STACK:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(MOVE_TO_CONE_STACK);
                        stack++;
                        sleep(200);
                        elevator.moveLift(Constants.elevatorPos.LOW);
                        sleep(200);
                        nextTrajectory(Constants.autoActions.BACK_TO_HIGH_JUNC);
                    }
                    break;
                case BACK_TO_HIGH_JUNC:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(BACK_TO_HIGH_JUNC);
                        nextTrajectory(Constants.autoActions.SCORE);
                    }
                    break;
                case MOVE_TO_PARK_LINE:
                    if (!drivetrain.isBusy()) {
                        drivetrain.followTrajectory(MOVE_TO_PARK_LINE);
                        nextTrajectory(Constants.autoActions.PARK);
                    }
                case PARK:
                    if (!drivetrain.isBusy()) {
//                        drivetrain.followTrajectory(PARK);
                        elevator.moveLift(Constants.elevatorPos.GROUND);
                        claw.moveClaw(Constants.clawState.OPEN);
                        nextTrajectory(Constants.autoActions.IDLE);
                    }
                    break;
                case IDLE:
                    if (!drivetrain.isBusy()) {
                        elevator.moveLift(Constants.elevatorPos.GROUND);
                        claw.moveClaw(Constants.clawState.OPEN);
                    }


            }

        }
    }

}
