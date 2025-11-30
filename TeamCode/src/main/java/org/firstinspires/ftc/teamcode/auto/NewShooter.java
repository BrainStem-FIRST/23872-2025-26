package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;

@Autonomous (name = "Auto")
public final class NewShooter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(58.8, 16.5, Math.toRadians(167.9));
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this, beginPose);

        waitForStart();

        if(isStopRequested())return;;



    }
}
