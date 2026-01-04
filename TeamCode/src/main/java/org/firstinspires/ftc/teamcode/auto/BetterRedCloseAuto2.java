package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.pidDrive.MathUtils.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.pidDrive.Waypoint;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name="BetterRedCloseAuto2")
@Config
public class BetterRedCloseAuto2 extends LinearOpMode {
    public static double[] start = new double[] { -62.5, 41, 0 };
    public static double[] close1Shooting = new double[] { -35, 35, 135 };
    public static double[] collect1Pre = new double[] { -12, 20, 90 };
    public static double collect1PreSlowDown = 0.4;
    public static double[] collect1 = new double[] { -12, 56, 90 };
//    public static double[] collect2 = new double[] {  };
//    //add pos 2
//    public static double[] collect2Pre = new double[] {  };
//    //add pre pos 2
//    public static double[] collect3 = new double[] {  };
//    //add pos 3
//    public static double[] collect3Pre = new double[] {  };
    //add pre pos 3
    public static double collectMaxPower = 0.3;
    BrainSTEMRobot robot;

    public SequentialAction ShootingSequence() {
        return new SequentialAction(
                AutoActions.fingerServoU(),
                new SleepAction(1.2),
                AutoActions.moveSpindexer120(),
                new SleepAction(1),
                AutoActions.fingerServoU(),
                new SleepAction(1.2),
                AutoActions.moveSpindexer120(),
                new SleepAction(1),
                AutoActions.fingerServoU(),
                new SleepAction(1.2),
                AutoActions.moveSpindexer60(),
                AutoActions.turnShooterOnIdle()
        );
    }

    public SequentialAction CollectingSequence() {
        return new SequentialAction(
                new SleepAction(1),
                AutoActions.moveSpindexer120(),
                new SleepAction(1),
                AutoActions.moveSpindexer120()
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20); // faster updates

        robot = new BrainSTEMRobot(hardwareMap, telemetry, this, createPose(start));
        AutoActions.setRobot(robot);

        DrivePath driveToPreloadShoot = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(close1Shooting))
        );
        DrivePath driveToCollectFirstSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect1Pre)).setSlowDownPercent(collectMaxPower),
                new Waypoint(createPose(collect1)).setMaxLinearPower(0.2)
        );

//        DrivePath driveToCollectSecondSpike = new DrivePath(robot.drive, telemetry,
//                new Waypoint(createPose(collect2Pre)).setSlowDownPercent(collect1PreSlowDown),
//                new Waypoint(createPose(collect2)).setMaxLinearPower(collectMaxPower)
//        );
//        DrivePath driveToCollectThirdSpike = new DrivePath(robot.drive, telemetry,
//                new Waypoint(createPose(collect3Pre)).setSlowDownPercent(collect1PreSlowDown),
//                new Waypoint(createPose(collect3)).setMaxLinearPower(collectMaxPower)
//        );

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                AutoActions.turnShooterOnIdle(),
                                // drive to shooting
//                                driveToPreloadShoot,
                                // turn shooter on
                                AutoActions.shooterTurnOnClose(),
                                // shoot 3
                                ShootingSequence(),
                                // collect first spike
                                new ParallelAction(
//                                        driveToCollectFirstSpike,
                                        AutoActions.setCollectorOn(),
                                        new SequentialAction(
                                                CollectingSequence()
                                        )
                                ),
                                AutoActions.shooterTurnOnClose(),
//                                driveToPreloadShoot,
                                AutoActions.shooterTurnOnClose(),
                                AutoActions.moveSpindexer60(),
                                new SleepAction(0.15),
                                ShootingSequence()

                        ),
                        AutoActions.robotUpdate()
        ));
    }


}