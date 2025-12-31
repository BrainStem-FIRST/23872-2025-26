//package org.firstinspires.ftc.teamcode.auto;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.BrainSTEMAutoRobot;
//import org.firstinspires.ftc.teamcode.auto_subsystems.AutoActions;
//import org.firstinspires.ftc.teamcode.auto_subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.auto_subsystems.Spindexer;
//
//@Autonomous (name = "Blue Far Auto")
//@Config
//public final class BlueFarAuto extends LinearOpMode {
//    public static class Positions {
//        public double startX = -62.6, startY = -16.6, startA = Math.toRadians(180);
//        public double preloadX = -49, preloadY = -11, preloadA = Math.toRadians(215), preloadT = Math.toRadians(215);
//        public double collect1X = 36, collect1Y = -32, collect1A = Math.toRadians(275), collect1T = Math.toRadians(275);
//        public double collect2X = -10, collect2Y = -42.5, collect2A = Math.toRadians(275), collect2T = Math.toRadians(275);
//
//    }
//    public static Positions positions = new Positions();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // DECLARE POSES
//        Pose2d beginPose = new Pose2d(positions.startX, positions.startY, positions.startA);
//        Pose2d shootPose = new Pose2d(positions.preloadX, positions.preloadY, positions.preloadA);
//        Pose2d collectPose = new Pose2d(positions.collect1X, positions.collect1Y, positions.collect1A);
//        Pose2d collectPose2 = new Pose2d(positions.collect2X, positions.collect2Y, positions.collect2A);
//
//        BrainSTEMAutoRobot robot = new BrainSTEMAutoRobot(hardwareMap, telemetry, this, beginPose);
//
//        Action preloadDrive = robot.drive.actionBuilder(beginPose)
//                .splineToLinearHeading(shootPose, positions.preloadT)
//                .build();
//
//        Action collectDrive = robot.drive.actionBuilder(shootPose)
//                .splineToLinearHeading(collectPose, positions.collect1T)
//                .build();
//
//        Action collectDrive2 = robot.drive.actionBuilder(collectPose)
//                .splineToLinearHeading(collectPose2, positions.collect2T)
//                .build();
//
//        Action robotUpdate = new AutoActions().robotUpdate(robot);
//
//        Action moveSpindexer120 = new AutoActions().moveSpindexer120(robot);
//
//        Action moveSpindexer60 = new AutoActions().moveSpindexer60(robot);
//
//        Action shooterTurnOnFar = new AutoActions().shooterTurnOnFar(robot);
//
//        Action shooterTurnOff= new AutoActions().shooterTurnOff(robot);
//
//        Action fingerServoU = new AutoActions().fingerServoU(robot);
//
//        Action fingerServoD = new AutoActions().fingerServoD(robot);
//
//
//
//
//        waitForStart();
//
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        new Action[]{new SequentialAction(
//                                preloadDrive,
//                                shooterTurnOnFar,
//                                new SleepAction(2),
//                                fingerServoU,
//                                new SleepAction(2),
//                                fingerServoD,
//                                new SleepAction(2),
//                                moveSpindexer120,
//                                new SleepAction(2),
//                                fingerServoU,
//                                new SleepAction(2),
//                                fingerServoD,
//                                new SleepAction(2),
//                                moveSpindexer120,
//                                new SleepAction(2),
//                                fingerServoU,
//                                new SleepAction(2),
//                                fingerServoD,
//                                shooterTurnOff
//
////                                preloadDrive,
////                                collectDrive,
////                                collectDrive2
//
//
//                        ),
//                                robotUpdate})
//
//        );
//
//        while (opModeIsActive());
//
//
////            robot.shooter.setShooterShootFar();
////            robot.update();
////
////
////            wait(750);
////
////            robot.finger.fingerState = Finger.FingerState.UP;
////            robot.update();
////
////            wait(1000);
////
////            robot.finger.fingerState = Finger.FingerState.DOWN;
////            robot.update();
////
////            robot.shooter.setShooterOff();
////            robot.update();
////
////            robot.spindexer.rotateDegrees(60);
////            robot.update();
////
////
////        wait (100);
////
//        robot.update();
//
//
//        while (opModeIsActive());
//
//
//
//
//
//    }
//
////    public void wait(double time, BrainSTEMTeAutoRobot robot){
////        Actions.runBlocking(
////                robot.drive.actionBuilder(robot.drive.localizer.getPose())
////                        .waitSeconds(time)
////                        .build());
////    }
//
//}
