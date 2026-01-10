package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.utils.pidDrive.UtilFunctions.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

@Autonomous(name="untested Blue Close")
@Config
public class MinBlueClose extends LinearOpMode {
    public static double[] start = new double[] { -62.5, -41, 0 };

    //1st Spike!!
    public static double[] close1Shooting = new double[] {-24, -24, -135};
    public static double[] collect1Pre = new double[] { -12, -34, -90 };
    public static double[] collect1Mid = new double[] { -12, -22, -90 };
    public static double[] collect1 = new double[] { -13, -40, -90 };
    public static double[] collect2 = new double[] { -13, -45, -90 };
    public static double[] collect3 = new double[] { -13, -50, -90 };
    public static double[] strafePos = new double[] { -36, -17, -90 };

    //2nd spike!!
    public static double[] collect2Pre = new double[] { 12, -28, -90 };
    public static double[] collect2Mid = new double[] { 12, -22, -90 };

    public static double[] collect4 = new double[] { 12, -40, -90 };
    public static double[] collect5 = new double[] { 12, -45, -90 };
    public static double[] collect6 = new double[] { 12, -50, -90 };
    public static double collectMaxPower = 0.3;
    BrainSTEMRobot robot;

    private static class PARAMS{

        // Comes after collect a spike and before spindexing
        private double COLLECT_TO_SPIND_WAIT = 0.8;
        // Comes after spindexer and before drive
        private double SPIND_TO_DRIVE_WAIT = 0.3;

        // Max power for collecting artifacts
        private double COLLECT_DRIVE_MAX_POWER = 0.15;
    }
    public static MinBlueClose.PARAMS PARAMS = new MinBlueClose.PARAMS();

    public SequentialAction ShootingSequence() {
        return new SequentialAction(
                AutoActions.fingerServoU(),
                new SleepAction(0.4),
                AutoActions.moveSpindexer120(),
                new SleepAction(0.2),
                AutoActions.fingerServoU(),
                new SleepAction(0.4),
                AutoActions.moveSpindexer120(),
                new SleepAction(0.2),
                AutoActions.fingerServoU(),
                new SleepAction(0.4),
                AutoActions.moveSpindexer60(),
                AutoActions.turnShooterOnIdle()
        );
    }

    public SequentialAction CollectingSequence() {
        return new SequentialAction(
                new SleepAction(0.5),
                AutoActions.moveSpindexer120(),
                new SleepAction(0.5),
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
                new Waypoint(createPose(close1Shooting)).setMaxLinearPower(1)
        );

        //1st Spike ===================================================================
        DrivePath driveToCollect1Pre = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect1Mid)).setSlowDownPercent(0.3),
                new Waypoint(createPose(collect1Pre)).setSlowDownPercent(0.1)
        );
        DrivePath driveToCollectFirstSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect1)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );

        DrivePath driveToCollectSecondSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );
        DrivePath driveToCollectThirdSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect3)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );
        DrivePath driveOffLine = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(strafePos)).setMaxLinearPower(0.5)
        );

        //2nd Spike!! ===================================================================
        DrivePath driveToCollect2Pre = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2Pre)).setSlowDownPercent(0.3),
                new Waypoint(createPose(collect2Pre)).setSlowDownPercent(0.1)
        );
        DrivePath driveToCollectFourthSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect4)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );

        DrivePath driveToCollectFifthSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect5)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );
        DrivePath driveToCollectSixthSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect6)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                // PRELOAD SHOOT
                                new ParallelAction(
                                        AutoActions.shooterTurnOnClose(),
                                        driveToPreloadShoot,
                                        AutoActions.waitForAccurateShooterVelocity()
                                ),
                                ShootingSequence(),

                                // 1ST SPIKE
                                AutoActions.setCollectorOn(),
                                driveToCollect1Pre,

                                // Ball 1
                                new ParallelAction(
                                        driveToCollectFirstSpike,
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                AutoActions.moveSpindexer120()
                                        )
                                ),

                                new SleepAction(0.2),

                                // Ball 2
                                new ParallelAction(
                                        driveToCollectSecondSpike,
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                AutoActions.moveSpindexer120()
                                        )
                                ),

                                new SleepAction(0.2),

                                // Ball 3
                                new ParallelAction(
                                        driveToCollectThirdSpike,
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                AutoActions.moveSpindexer120()
                                        )
                                ),

                                new SleepAction(0.2),

                                // Trans to shoot
                                new ParallelAction(
                                        AutoActions.setCollectorOff(),
                                        AutoActions.moveSpindexer60(),
                                        AutoActions.shooterTurnOnClose(),
                                        driveToPreloadShoot
                                ),
                                AutoActions.waitForAccurateShooterVelocity(),
                                ShootingSequence(),

                                // 2ND SPIKE
                                AutoActions.setCollectorOn(),
                                driveToCollect2Pre,

                                // Ball 4
                                new ParallelAction(
                                        driveToCollectFourthSpike,
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                AutoActions.moveSpindexer120()
                                        )
                                ),

                                new SleepAction(0.2),

                                // Ball 5
                                new ParallelAction(
                                        driveToCollectFifthSpike,
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                AutoActions.moveSpindexer120()
                                        )
                                ),

                                new SleepAction(0.2),

                                // Ball 6
                                new ParallelAction(
                                        driveToCollectSixthSpike,
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                AutoActions.moveSpindexer120()
                                        )
                                ),

                                // trans to shoot
                                new ParallelAction(
                                        AutoActions.moveSpindexer60(),
                                        AutoActions.shooterTurnOnClose(),
                                        driveToPreloadShoot
                                ),
                                AutoActions.waitForAccurateShooterVelocity(),
                                ShootingSequence(),

                                driveOffLine


                        ),
                        AutoActions.robotUpdate(telemetry)
                ));
    }



}