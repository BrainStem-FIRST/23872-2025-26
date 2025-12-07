package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.BrainSTEMAutoRobot;
import org.firstinspires.ftc.teamcode.auto_subsystems.Collector;

import org.firstinspires.ftc.teamcode.auto_subsystems.AutoActions;
import org.firstinspires.ftc.teamcode.auto_subsystems.Finger;
import org.firstinspires.ftc.teamcode.auto_subsystems.Spindexer;


@Autonomous (name = "Auto")
@Config
public final class BasicAuto extends LinearOpMode {
    BrainSTEMAutoRobot robot;

    public static class Positions {
        public double startX = 62.6, startY = 16.6, startA = Math.toRadians(180);
        public double preloadX = 49, preloadY = 11, preloadA = Math.toRadians(145), preloadT = Math.toRadians(145);
        public double collect1_1X = 38, collect1_1Y = 22, collect1_1A = Math.toRadians(90), collect1_1T = Math.toRadians(90);
        public double collect1_2X = 38, collect1_2Y = 24, collect1_2A = Math.toRadians(90), collect1_2T = Math.toRadians(90);
        public double collect1_3X = 38, collect1_3Y = 27 , collect1_3A = Math.toRadians(90), collect1_3T = Math.toRadians(90);

    }
    public Positions positions = new Positions();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(positions.startX, positions.startY, positions.startA);
        BrainSTEMAutoRobot robot = new BrainSTEMAutoRobot(hardwareMap, telemetry, this, beginPose);

        robot = new BrainSTEMAutoRobot(hardwareMap, telemetry, this, beginPose);

        // DECLARE POSES

        Pose2d shootPose = new Pose2d(positions.preloadX, positions.preloadY, positions.preloadA);
        Pose2d collectPose = new Pose2d(positions.collect1_1X, positions.collect1_1Y, positions.collect1_1A);
        robot.collector.collectorState = Collector.CollectorState.ON;
        wait(500);
        robot.collector.collectorState = Collector.CollectorState.OFF;
        Pose2d collectPose2 = new Pose2d(positions.collect1_2X, positions.collect1_2Y, positions.collect1_2A);
        Pose2d collectPose3 = new Pose2d(positions.collect1_3X, positions.collect1_3Y, positions.collect1_3A);




        Action preloadDrive = robot.drive.actionBuilder(beginPose)
                .splineToLinearHeading(shootPose, positions.preloadT)
                .build();

        Action collectDrive = robot.drive.actionBuilder(shootPose)
                .splineToLinearHeading(collectPose, positions.collect1_1T)
                .build();

        Action collectDrive2 = robot.drive.actionBuilder(collectPose)
                .splineToLinearHeading(collectPose2, positions.collect1_2T)
                .build();

        Action collectDrive3 = robot.drive.actionBuilder(collectPose2)
                .splineToLinearHeading(collectPose3, positions.collect1_3T)
                .build();

        Action setIndex1 = new AutoActions().setIndex1(robot);
        Action robotUpdate = new AutoActions().robotUpdate(robot);

        Action setIndex2 = new AutoActions().setIndex2(robot);

        Action setIndex3 = new AutoActions().setIndex3(robot);
        Action rotate120 = new AutoActions().rotate120(robot);
        waitForStart();


            if (isStopRequested()) return;

//            robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT1;

        Actions.runBlocking(
                new ParallelAction(
                       new SequentialAction(

                                setIndex1,
                                new SleepAction(1.0),
                                rotate120
//                                setIndex2
//                               new SleepAction(1.0)


//                                preloadDrive,
//                                collectDrive,
//                                collectDrive2,
//                                collectDrive3

                        ), robotUpdate)

        );


//            robot.shooter.setShooterShootFar();
//            robot.update();
//
//
//            wait(750);
//
//            robot.finger.fingerState = Finger.FingerState.UP;
//            robot.update();
//
//            wait(1000);
//
//            robot.finger.fingerState = Finger.FingerState.DOWN;
//            robot.update();
//
//            robot.shooter.setShooterOff();
//            robot.update();
//
//            robot.spindexer.rotateDegrees(60);
//            robot.update();
//
//
//        wait (100);
//
        robot.update();


        while (opModeIsActive()) ;


    }



}
