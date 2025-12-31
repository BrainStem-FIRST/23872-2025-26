package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.firstinspires.ftc.teamcode.auto_subsystems.Collector;
import org.firstinspires.ftc.teamcode.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.pidDrive.Waypoint;


@Config
@Autonomous(name="Blue Close Auto")
public class BlueCloseAuto extends LinearOpMode {
    public static double x = 24, y = 0, h = 0;
    public static double maxPower = 0.5;

    BrainSTEMAutoRobot robot;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Coordinates coordinates = new Coordinates(false);
        Pose2d start = coordinates.getCloseStartPose();
        Pose2d end = new Pose2d(x, y, h);


        robot = new BrainSTEMAutoRobot(hardwareMap, telemetry, this, start);
        DrivePath driveToPreloadShoot = new DrivePath(robot.drive, telemetry,
                new Waypoint(coordinates.getClose1ShootingPose())
        );
        DrivePath driveToCollectFirstSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(coordinates.getClose1CollectPrePose())
        );
        DrivePath driveToCollect1 = new DrivePath(robot.drive, telemetry,
                new Waypoint(coordinates.getCollect1Pose())
        );
        DrivePath driveToCollect2 = new DrivePath(robot.drive, telemetry,
                new Waypoint(coordinates.getCollect2Pose())
        );
        DrivePath driveToCollect3 = new DrivePath(robot.drive, telemetry,
                new Waypoint(coordinates.getCollect3Pose())
        );

        Action setCollectorOn = new AutoActions().setCollectorOn(robot);
        Action moveSpindexer120 = new AutoActions().moveSpindexer120(robot);
        Action moveSpindexer60 = new AutoActions().moveSpindexer60(robot);
        Action updateRobot = new AutoActions().robotUpdate(robot);
        Action setShooterIdle = new AutoActions().shooterTurnOnIdle(robot);
        Action setShooterClose = new AutoActions().shooterTurnOnClose(robot);
        Action fingerUp = new AutoActions().fingerServoU(robot);
        Action fingerDown = new AutoActions().fingerServoD(robot);


        // READ THIS INFO VERY CAREFULLY
        // the robot starts at "start" and will drive to "end" and then drive back to "start"
        // why will it do this? because when I declare the "path" variable, I pass in TWO Waypoint objects
        // Whenever you declare a waypoint, you need to pass in the pose of the waypoint
        // the drivetrain will follow the waypoints in the order you declare them
        // if you want to create another drive path then just do:
        // DrivePath path2 = new DrivePath(robot.drive, etc pass in your waypoints here...)

        // for right now, there are 5 values you need to tune
        // 2 of them are located in PathParams (inside this file you will see a class called DefaultParams; its in this inner class)
        //   farHeadingKp: if you increase it, the robot will rotate faster but will oscillate more (vice versa)
        //   speedKp: if you increase it, the robot will drive faster but will oscillate more (and vice versa)
        // the other 3 values you need to tune are in Tolerance class
        //   xTol, yTol, and headingTol
        // the robot won't end the path until the position and heading errors are within these tolerances (so I suggest setting them to something like 3, 3, and Math.toRadians(5))

        // how do you know when you're done tuning? idk you decide, when the robot is not oscillating at each waypoint anymore
        waitForStart();
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                // put auto stuff here please
                                setShooterClose,
                                driveToPreloadShoot,

                                // shoot stuff
                                fingerUp,
                                new SleepAction(1.5),
                                fingerDown,
                                new SleepAction(0.3),
                                moveSpindexer120,
                                fingerUp,
                                new SleepAction(1.5),
                                fingerDown,
                                new SleepAction(0.3),
                                moveSpindexer120,
                                fingerUp,
                                new SleepAction(1.5),
                                fingerDown,
                                new SleepAction(0.3),
                                moveSpindexer120,
                                setShooterIdle,

                                new ParallelAction(
                                        driveToCollectFirstSpike,
                                        setCollectorOn
                                ),
                                driveToCollect1,

                                // collect sequence
                                moveSpindexer60,
                                new SleepAction(0.75),
                                driveToCollect2,
                                new SleepAction(0.75),
                                moveSpindexer120,
                                new SleepAction(0.75),
                                driveToCollect3,
                                new SleepAction(0.75),
                                moveSpindexer60,
                                setShooterClose,
                                driveToPreloadShoot,
                                fingerUp,
                                new SleepAction(1.5),
                                fingerDown,
                                new SleepAction(0.3),
                                moveSpindexer120,
                                fingerUp,
                                new SleepAction(1.5),
                                fingerDown,
                                new SleepAction(0.3),
                                moveSpindexer120,
                                fingerUp,
                                new SleepAction(1.5),
                                fingerDown,
                                new SleepAction(0.3),
                                moveSpindexer120,
                                setShooterIdle
                        ),
                        updateRobot
                ));
    }
}
