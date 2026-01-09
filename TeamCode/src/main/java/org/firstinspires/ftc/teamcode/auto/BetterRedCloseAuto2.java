package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.pidDrive.MathUtils.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

@Autonomous(name="Seneca Valley Auto")
@Config
public class BetterRedCloseAuto2 extends LinearOpMode {
    public static double[] start = new double[] { -62.5, 41, 0 };

    //1st Spike!!
    public static double[] close1Shooting = new double[] {-24, 24, 135 };
    public static double[] collect1Pre = new double[] { -12, 28, 90 };
    public static double[] collect1Mid = new double[] { -12, 22, 90 };
    public static double[] collect1 = new double[] { -13, 34, 90 };
    public static double[] collect2 = new double[] { -13, 42, 90 };
    public static double[] collect3 = new double[] { -13, 47, 90 };
    public static double[] strafePos = new double[] { -36, 14, 90 };

    //2nd spike!!
    public static double[] collect2Pre = new double[] { -12, 28, 90 };
    public static double[] collect2Mid = new double[] { -12, 22, 90 };

    public static double[] collect4 = new double[] { 12, 34, 90 };
    public static double[] collect5 = new double[] { 12, 42, 90 };
    public static double[] collect6 = new double[] { 12, 47, 90 };
    public static double collectMaxPower = 0.3;
    BrainSTEMRobot robot;

    private static class PARAMS{

        // Comes after collect a spike and before spindexing
        private double COLLECT_TO_SPIND_WAIT = 0.55;
        // Comes after spindexer and before drive
        private double SPIND_TO_DRIVE_WAIT = 0.3;

        // Max power for collecting artifacts
        private double COLLECT_DRIVE_MAX_POWER = 0.15;
    }
    public static BetterRedCloseAuto2.PARAMS PARAMS = new BetterRedCloseAuto2.PARAMS();

    public SequentialAction ShootingSequence() {
        return new SequentialAction(
                AutoActions.fingerServoU(),
                new SleepAction(0.4),
                AutoActions.moveSpindexer120(),
                new SleepAction(0.3),
                AutoActions.fingerServoU(),
                new SleepAction(0.4),
                AutoActions.moveSpindexer120(),
                new SleepAction(0.3),
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
                new Waypoint(createPose(collect1Mid)).setSlowDownPercent(0.3),
                new Waypoint(createPose(collect1Pre)).setSlowDownPercent(0.1)
        );
        DrivePath driveToCollectFourthSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect1)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );

        DrivePath driveToCollectFifthSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );
        DrivePath driveToCollectSixthSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect3)).setMaxLinearPower(PARAMS.COLLECT_DRIVE_MAX_POWER).setMaxTime(3)
        );

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                // Ramp up shooter and drive to preload shoot
                                AutoActions.shooterTurnOnClose(),
                                new ParallelAction(
                                        AutoActions.waitForAccurateShooterVelocity(),
                                        driveToPreloadShoot
                                ),

                                // "Domino Sequence"
                                ShootingSequence(),

                                AutoActions.setCollectorOn(),
                                new SleepAction(0.3),

                                //1st Spike Does Work ==========================
                                driveToCollect1Pre,
                                driveToCollectFirstSpike,
//                                AutoActions.setCollectorOff(),
                                new SleepAction(PARAMS.COLLECT_TO_SPIND_WAIT),
                                AutoActions.moveSpindexer120(),
                                new SleepAction(PARAMS.SPIND_TO_DRIVE_WAIT),
                                driveToCollectSecondSpike,
                                new SleepAction(PARAMS.COLLECT_TO_SPIND_WAIT),
                                AutoActions.moveSpindexer120(),
                                new SleepAction(PARAMS.SPIND_TO_DRIVE_WAIT),
                                driveToCollectThirdSpike,
                                new SleepAction(PARAMS.COLLECT_TO_SPIND_WAIT),
                                AutoActions.moveSpindexer120(),
                                new SleepAction(PARAMS.SPIND_TO_DRIVE_WAIT),

                                new ParallelAction(
                                        AutoActions.setCollectorOff(),
                                        AutoActions.moveSpindexer60()
                                ),


                                // Shooting sequence for 1st spike
                                // Turn shooter back on
                                AutoActions.shooterTurnOnClose(),
                                // Drive to shoot position
                                driveToPreloadShoot,
                                new SleepAction(0.3),
                                // Last shooting sequence
                                AutoActions.waitForAccurateShooterVelocity(),
                                ShootingSequence(),
                                new SleepAction(0.3),

                                //2nd Spike May Not Work ==========================
                                AutoActions.setCollectorOn(),
                                new SleepAction(0.3),
                                driveToCollect2Pre,
                                driveToCollectFourthSpike,
                                new SleepAction(PARAMS.COLLECT_TO_SPIND_WAIT),
                                AutoActions.moveSpindexer120(),
                                new SleepAction(PARAMS.SPIND_TO_DRIVE_WAIT),
                                driveToCollectFifthSpike,
                                new SleepAction(PARAMS.COLLECT_TO_SPIND_WAIT),
                                AutoActions.moveSpindexer120(),
                                new SleepAction(PARAMS.SPIND_TO_DRIVE_WAIT),
                                driveToCollectSixthSpike,
                                new SleepAction(PARAMS.COLLECT_TO_SPIND_WAIT),
                                AutoActions.moveSpindexer120(),
                                new SleepAction(PARAMS.SPIND_TO_DRIVE_WAIT),
                                driveOffLine


                        ),
                        AutoActions.robotUpdate(telemetry)
        ));
    }



}