//package org.firstinspires.ftc.teamcode.opModes;
//
////TELEMETRY & DASHBOARD
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//
////ROADRUNNER
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//
////UTIL
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.subsystems.Constants;
//
////SUBSYSTEMS
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.Claw;
//import org.firstinspires.ftc.teamcode.subsystems.Elevator;
//import org.firstinspires.ftc.teamcode.subsystems.Kamera;
//
//@Autonomous(name = "RoadRunner Experimental")
//@Config
//public class rrEx extends LinearOpMode {
//
//    ElapsedTime runtime = new ElapsedTime();
//    Constants.autoActions currentTrajectory = Constants.autoActions.IDLE;
//
//    SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
//    Elevator elevator = new Elevator();
//    Claw claw = new Claw();
//    Kamera camera = new Kamera();
//
//    boolean clawState;
//    double elevatorPosition;
//
//    Pose2d startPos = new Pose2d(60, 36, Math.toRadians(90));
//    Pose2d coneStackPos;
//
//    int stack = 5;
//    int high = 0;
//    int mid = 0;
//    int low = 0;
//
//    void nextTrajectory(Constants.autoActions a){
//        double time = runtime.seconds();
//        currentTrajectory = a;
//    }
//
//    @Override
//    public void runOpMode() {
//        drivetrain.setPoseEstimate(startPos);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        //TRAJECTORIES
//        Trajectory TO_HIGHJUNC = drivetrain.trajectoryBuilder(startPos)
//                //MOVE TO FIRST HIGH JUNC (24, 0)
//                //LIFT ELEVATOR TO HIGH POSITION
//                .lineTo(new Vector2d(0, 36))
//                .addDisplacementMarker(
//                        3,
//                        () -> {
//                            elevator.moveLift(Constants.elevatorPos.HIGH);
//                        })
//                .lineToLinearHeading(new Pose2d(-32, 0, Math.toRadians(0)))
//                .build();
//
//        //Trajectory SCORE_PRELOAD = drivetrain.trajectoryBuilder(TO_HIGHJUNC.end())
//
//        runtime.reset();
//
//        camera.init(hardwareMap);
//        claw.init(hardwareMap);
//        elevator.init(hardwareMap);
//
//        clawState = claw.clawState;
//        //elevatorPosition = elevator.getPosition();
//
//        Vector2d parkZone;
//        int zone = -1;
//
//        claw.moveClaw(Constants.clawState.CLOSE);
//
//        while (opModeInInit())
//        {
//            zone = camera.getPosition();
//            telemetry.addData("Parking Zone: ", zone);
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        if (zone == 1){
//            parkZone = new Vector2d(12, -12);
//        }
//        else if (zone == 2){
//            parkZone = new Vector2d(36, -12);
//        }
//        else if (zone == 3){
//            parkZone = new Vector2d(60, -12);
//        }
//        else if (zone == -1) {
//            parkZone = new Vector2d(12, -12);
//        }
//
//        //PARK TRAJECTORY BASED ON ZONE
//        //trajectory PARK
//
//        telemetry.addLine("AUTONOMOUS READY...");
//        telemetry.update();
//
//        currentTrajectory = Constants.autoActions.TO_HIGH_JUNC;
//
//        while (opModeIsActive()) {
//            switch(currentTrajectory){
//                case TO_HIGHJUNC:
//                    if (!drivetrain.isBusy()) {
//                        drivetrain.followTrajectory(TO_HIGHJUNC);
//                        nextTrajectory(currentTrajectory.SCORE);
//                    }
//                    break;
//            }
//
//            telemetry.addData("Current Trajectory", currentTrajectory);
//            telemetry.addLine("");
//            telemetry.addLine("true - CLOSE     false - OPEN");
//            telemetry.addData("Claw State: ", clawState);
//            telemetry.addData("Elevator Position", elevator.getPosition());
//            telemetry.addLine("");
//            telemetry.addData("Zone: ", zone);
//            telemetry.update();
//        }
//    }
//
//
//}
