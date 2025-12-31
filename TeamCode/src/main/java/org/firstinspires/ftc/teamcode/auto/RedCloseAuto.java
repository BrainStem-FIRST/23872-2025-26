//package org.firstinspires.ftc.teamcode.auto;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
//import org.firstinspires.ftc.teamcode.auto.AutoActions;
//import org.firstinspires.ftc.teamcode.auto_subsystems.Collector;
//import org.firstinspires.ftc.teamcode.pidDrive.DrivePath;
//import org.firstinspires.ftc.teamcode.pidDrive.Waypoint;
//
//@Config
//@Autonomous(name="RedCloseAuto")
//public class RedCloseAuto extends LinearOpMode {
//    public static double x = 24, y = 0, h = 0;
//    public static double maxPower = 0.5;
//
//    BrainSTEMAutoRobot robot;
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Pose2d start = new Pose2d(-63, 36, 0);
//        Pose2d end = new Pose2d(x, y, h);
//
//        Pose2d close1ShootingPose = new Pose2d(-15, 22, Math.toRadians(135));
//        Pose2d close1CollectPrePose = new Pose2d(-13, 28, Math.toRadians(90));
//        Pose2d collect1Pose = new Pose2d(-13, 43, Math.toRadians(90));
//        Pose2d collect2Pose = new Pose2d(-13, 48, Math.toRadians(90));
//        Pose2d collect3Pose = new Pose2d(-13, 53, Math.toRadians(90));
//        Pose2d close2ShootingPose = new Pose2d(-15, 22, Math.toRadians(135));
//
//
//        robot = new BrainSTEMAutoRobot(hardwareMap, telemetry, this, start);
//        DrivePath driveToPreloadShoot = new DrivePath(robot.drive, telemetry,
//                new Waypoint(close1ShootingPose)
//        );
//        DrivePath driveToCollectFirstSpike = new DrivePath(robot.drive, telemetry,
//                new Waypoint(close1CollectPrePose)
//        );
//        DrivePath driveToCollect1 = new DrivePath(robot.drive, telemetry,
//                new Waypoint(collect1Pose)
//        );
//        DrivePath driveToCollect2 = new DrivePath(robot.drive, telemetry,
//                new Waypoint(collect2Pose)
//        );
//        DrivePath driveToCollect3 = new DrivePath(robot.drive, telemetry,
//                new Waypoint(collect3Pose)
//        );
//
//        Action setCollectorOn = new AutoActions().setCollectorOn(robot);
//        Action moveSpindexer120 = new AutoActions().moveSpindexer120(robot);
//        Action moveSpindexer60 = new AutoActions().moveSpindexer60(robot);
//        Action updateRobot = new AutoActions().robotUpdate(robot);
//
//
//        // READ THIS INFO VERY CAREFULLY
//        // the robot starts at "start" and will drive to "end" and then drive back to "start"
//        // why will it do this? because when I declare the "path" variable, I pass in TWO Waypoint objects
//        // Whenever you declare a waypoint, you need to pass in the pose of the waypoint
//        // the drivetrain will follow the waypoints in the order you declare them
//        // if you want to create another drive path then just do:
//        // DrivePath path2 = new DrivePath(robot.drive, etc pass in your waypoints here...)
//
//        // for right now, there are 5 values you need to tune
//        // 2 of them are located in PathParams (inside this file you will see a class called DefaultParams; its in this inner class)
//        //   farHeadingKp: if you increase it, the robot will rotate faster but will oscillate more (vice versa)
//        //   speedKp: if you increase it, the robot will drive faster but will oscillate more (and vice versa)
//        // the other 3 values you need to tune are in Tolerance class
//        //   xTol, yTol, and headingTol
//        // the robot won't end the path until the position and heading errors are within these tolerances (so I suggest setting them to something like 3, 3, and Math.toRadians(5))
//
//        // how do you know when you're done tuning? idk you decide, when the robot is not oscillating at each waypoint anymore
//        waitForStart();
//        Actions.runBlocking(
//                new ParallelAction(
//                    new SequentialAction(
//                            // put auto stuff here please
//                        driveToPreloadShoot,
//                        new SleepAction(2),
//                        driveToCollectFirstSpike,
//                        setCollectorOn,
//                        driveToCollect1,
//                        moveSpindexer120,
//                        new SleepAction(2),
//                        driveToCollect2,
//                        new SleepAction(2),
//                        moveSpindexer120,
//                        new SleepAction(2),
//                        driveToCollect3,
//                        new SleepAction(2),
//                        moveSpindexer60
//                ),
//                updateRobot
//        ));
//    }
//}
