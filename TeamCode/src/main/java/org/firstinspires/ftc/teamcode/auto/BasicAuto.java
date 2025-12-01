package org.firstinspires.ftc.teamcode.auto;

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
public final class BasicAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(62.5, 15.8, Math.toRadians(177.8));
            BrainSTEMAutoRobot robot = new BrainSTEMAutoRobot(hardwareMap, telemetry, this, beginPose);

             

            Action setCollect1 = new AutoActions().setCollect1(robot);
            Action robotUpdate = new AutoActions().robotUpdate(robot);

            Action setCollect2 = new AutoActions().setCollect2(robot);

            Action setCollect3 = new AutoActions().setCollect3(robot);
            waitForStart();


            if (isStopRequested()) return;

            robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT1;

            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
//
//                                    setCollect1,
//                                    new SleepAction(1.0),
//                                    setCollect2
                                    robot.drive.actionBuilder(beginPose)

                                            .splineToSplineHeading(new Pose2d(30, 30, 0), 0)
                                            .build()















                            ),
                            robotUpdate
                    )

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
