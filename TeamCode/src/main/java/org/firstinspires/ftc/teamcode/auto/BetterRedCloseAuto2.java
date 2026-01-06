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

@Autonomous(name="Seneca Valley Auto")
@Config
public class BetterRedCloseAuto2 extends LinearOpMode {
    public static double[] start = new double[] { -62.5, 41, 0 };
    public static double[] close1Shooting = new double[] { -35, 35, 135 };
    public static double[] collect1Pre = new double[] { -12, 20, 90 };
    public static double collect1PreSlowDown = 0.4;
    public static double[] collect1 = new double[] { -12, 37, 90 };
    public static double[] collect2 = new double[] { -13, 42, 90 };
    public static double[] collect3 = new double[] { -13, 47, 90 };
//    //add pos 2
//    public static double[] collect2Pre = new double[] {  };
//    //add pos 3
//    public static double[] collect3Pre = new double[] {  };
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
                new Waypoint(createPose(close1Shooting))
        );
        DrivePath driveToCollectFirstSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect1Pre)).setSlowDownPercent(0.4),
                new Waypoint(createPose(collect1)).setMaxLinearPower(0.2)
        );

        DrivePath driveToCollectSecondSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2)).setMaxLinearPower(0.2)
        );
        DrivePath driveToCollectThirdSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect3)).setMaxLinearPower(0.2)
        );

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                // Ramp up shooter
                                AutoActions.turnShooterOnIdle(),
                                new SleepAction(5),
                                // Drive to preload shooter
                                driveToPreloadShoot,
                                // Turn shooter on
                                AutoActions.shooterTurnOnClose(),
                                // "Domino Sequence"
                                ShootingSequence(),
                                // Slow shooter for less battery drainage
                                AutoActions.turnShooterOnIdle(),
                                // Collection sequence
                                new ParallelAction(
                                        driveToCollectFirstSpike,
                                        AutoActions.setCollectorOn()

                                ),
                                new SleepAction(0.3),
                                AutoActions.moveSpindexer120(),
                                driveToCollectFirstSpike,
                                new SleepAction(1),
                                AutoActions.moveSpindexer120(),
                                new SleepAction(0.6),
                                driveToCollectSecondSpike,
                                new SleepAction(1),
                                AutoActions.moveSpindexer120(),
                                driveToCollectThirdSpike,
                                new SleepAction(1),
                                AutoActions.moveSpindexer120(),
                                // Turn shooter back on
                                AutoActions.shooterTurnOnClose(),
                                // Drive to shoot position
                                driveToPreloadShoot,
                                // Turn shooter on
                                AutoActions.shooterTurnOnClose(),
                                // Move spindexer
                                AutoActions.moveSpindexer60(),
                                new SleepAction(0.15)
                                // Last shooting sequence
//                                ShootingSequence()
                        ),
                        AutoActions.robotUpdate()
        ));
    }



}