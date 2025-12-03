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

import org.firstinspires.ftc.teamcode.auto_subsystems.AutoActions;
import org.firstinspires.ftc.teamcode.auto_subsystems.Finger;
import org.firstinspires.ftc.teamcode.auto_subsystems.Spindexer;

@Autonomous (name = "Auto")
@Config
public final class BasicAuto extends LinearOpMode {
    public static class Positions {
        public double startX = 62.6, startY = 16.6, startA = Math.toRadians(180);
        public double preloadX = 49, preloadY = 11, preloadA = Math.toRadians(145), preloadT = Math.toRadians(145);
        public double collect1X = 36, collect1Y = 32, collect1A = Math.toRadians(90), collect1T = Math.toRadians(90);
        public double collect2X = 10, collect2Y = 42.5, collect2A = Math.toRadians(90), collect2T = Math.toRadians(90);
    }
    public static Positions positions = new Positions();

    @Override
    public void runOpMode() throws InterruptedException {

        // DECLARE POSES
        Pose2d beginPose = new Pose2d(positions.startX, positions.startY, positions.startA);
        Pose2d shootPose = new Pose2d(positions.preloadX, positions.preloadY, positions.preloadA);
        Pose2d collectPose = new Pose2d(positions.collect1X, positions.collect1Y, positions.collect1A);

        BrainSTEMAutoRobot robot = new BrainSTEMAutoRobot(hardwareMap, telemetry, this, beginPose);

        Action preloadDrive = robot.drive.actionBuilder(beginPose)
             .splineToLinearHeading(shootPose, positions.preloadT)
                .build();

        Action collectDrive = robot.drive.actionBuilder(shootPose)
                    .splineToLinearHeading(collectPose, positions.collect1T)
                    .build();

        Action setCollect1 = new AutoActions().setCollect1(robot);
        Action robotUpdate = new AutoActions().robotUpdate(robot);

        Action setCollect2 = new AutoActions().setCollect2(robot);

        Action setCollect3 = new AutoActions().setCollect3(robot);
        waitForStart();

        
            if (isStopRequested()) return;

            robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT1;

            Actions.runBlocking(
                    new ParallelAction(
                            new Action[]{new SequentialAction(
//
//                                    setCollect1,
//                                    new SleepAction(1.0),
//                                    setCollect2

                                    preloadDrive,
                                    collectDrive


                            ), robotUpdate})

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


        while (opModeIsActive());





    }

//    public void wait(double time, BrainSTEMTeAutoRobot robot){
//        Actions.runBlocking(
//                robot.drive.actionBuilder(robot.drive.localizer.getPose())
//                        .waitSeconds(time)
//                        .build());
//    }

}
