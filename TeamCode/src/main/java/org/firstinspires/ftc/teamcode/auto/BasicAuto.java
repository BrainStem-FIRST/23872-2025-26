package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import subsystems.Collector;
import subsystems.Finger;
import subsystems.Spindexer;

@Autonomous (name = "Auto")
public final class BasicAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d beginPose = new Pose2d(58.8, 16.5, Math.toRadians(167.9));
            BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this, beginPose);



            waitForStart();


            if (isStopRequested()) return;


            robot.shooter.setShooterOn();
            robot.update();


            wait(2, robot);

            robot.finger.fingerState = Finger.FingerState.UP;
            robot.update();

            wait(1, robot);

        robot.finger.fingerState = Finger.FingerState.DOWN;
        robot.update();

        robot.shooter.setShooterOff();
        robot.update();

        robot.spindexer.rotateDegrees(60);
        robot.update();


        wait (0.1, robot);

        robot.update();


        while (opModeIsActive());






    }

    public void wait(double time, BrainSTEMRobot robot){
        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.localizer.getPose())
                        .waitSeconds(time)
                        .build());
    }


}
