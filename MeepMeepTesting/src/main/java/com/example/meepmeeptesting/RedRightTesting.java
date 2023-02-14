//package com.example.meepmeeptesting;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//
//public class RedRightTesting {
//    public static void main(String[] args) {
//        MeepMeep mm = new MeepMeep(600);
//
//        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(mm)
//                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(180), Math.toRadians(180), 12.73)
//                .setDimensions(14, 16)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(36, -60, Math.toRadians(90)))
//                                //move to high junction
//                                .lineTo(new Vector2d(36, 0))
//                                .lineToLinearHeading(new Pose2d(32, 0, Math.toRadians(180)))
//
//                                //return to in line
//                                .lineToLinearHeading(new Pose2d(36,0, Math.toRadians(90)))
//
//                                //move to in line with stack
//                                .lineTo(new Vector2d(36, -12))
//                                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
//
//                                //move back to in line
//                                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)))
//
//                                //move to high junction
//                                .lineTo(new Vector2d(36,0))
//                                .lineToLinearHeading(new Pose2d(32, 0, Math.toRadians(180)))
//
//                                //move back in line
//                                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(90)))
//
//                                //move to parking line
//                                .lineTo(new Vector2d(36, -12))
//
//                                //park zone 1
//                                .lineToConstantHeading(new Vector2d(12, -12))
//
//                                //park zone 3
////                                .lineToConstantHeading(new Vector2d(60, -12))
//                                .build()
//                );
//
//        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(mm)
//                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(180), Math.toRadians(180), 12.73)
//                .setDimensions(14, 16)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
//                                //move to high junction
//                                .lineTo(new Vector2d(-36, 0))
//                                .lineToLinearHeading(new Pose2d(-32, 0, Math.toRadians(0)))
//
//                                //move to stack
//                                .splineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)), Math.toRadians(180))
//
////                                //return to in line
////                                .lineToLinearHeading(new Pose2d(-36,0, Math.toRadians(90)))
////
////                                //move to in line with stack
////                                .lineTo(new Vector2d(-36, -12))
////                                .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)))
//
//                                //move back to HIGH_JUNC
//                                .splineToLinearHeading(new Pose2d(-32, 0, Math.toRadians(0)), Math.toRadians(0))
//
//                                //move back to in line
//                                //.lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
//
////                                //move to high junction
////                                .lineTo(new Vector2d(-36,0))
////                                .lineToLinearHeading(new Pose2d(-32, 0, Math.toRadians(0)))
//
//                                //move back in line
//                                .lineToLinearHeading(new Pose2d(-36, 0, Math.toRadians(90)))
//
//                                //move to parking line
//                                .lineTo(new Vector2d(-36, -12))
//
//                                //park zone 1
//                                .lineToConstantHeading(new Vector2d(-60, -12))
//
//                                //park zone 3
////                                .lineToConstantHeading(new Vector2d(-12, -12))
//                                .build()
//                );
//
//        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(1f)
//                .addEntity(bot1)
//                .addEntity(bot2)
//                .start();
//
//    }
//}