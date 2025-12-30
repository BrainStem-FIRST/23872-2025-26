package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMAutoRobot;
import org.firstinspires.ftc.teamcode.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.pidDrive.Waypoint;

@Config
@Autonomous(name="kevin example auto")
public class KevinExampleAuto extends LinearOpMode {
    public static double x = 24, y = 0, h = 0;
    public static double maxPower = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(0, 0, 0);
        Pose2d end = new Pose2d(x, y, h);

        BrainSTEMAutoRobot robot = new BrainSTEMAutoRobot(hardwareMap, telemetry, this, start);
        DrivePath path = new DrivePath(robot.drive,
                new Waypoint(end).setMaxPower(maxPower)
        );
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
        Actions.runBlocking(path);
    }
}
