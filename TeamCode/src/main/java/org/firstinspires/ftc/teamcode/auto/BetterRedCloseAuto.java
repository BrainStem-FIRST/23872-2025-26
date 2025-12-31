package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.pidDrive.MathUtils.createPose;

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
import org.firstinspires.ftc.teamcode.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.pidDrive.Waypoint;

@Autonomous(name="BetterRedCloseAuto")
@Config
public class BetterRedCloseAuto extends LinearOpMode {
    public static double[] start = new double[] { -63, 36, 0 };
    public static double[] close1Shooting = new double[] { -15, 22, 135 };
    public static double[] collect1Pre = new double[] { -13, 28, 90 };
    public static double collect1PreSlowDown = 0.4;
    public static double[] collect1 = new double[] { -13, 54, 90 };
    public static double collectMaxPower = 0.3;
    BrainSTEMRobot robot;

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
                new Waypoint(createPose(collect1Pre)).setSlowDownPercent(collect1PreSlowDown),
                new Waypoint(createPose(collect1)).setMaxLinearPower(collectMaxPower)
        );

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                // shoot preloads
                                driveToPreloadShoot,
                                new SleepAction(2),

                                // collect first spike
                                AutoActions.setCollectorOn(),
                                new ParallelAction(
                                        driveToCollectFirstSpike,
                                        new SequentialAction(
                                                AutoActions.moveSpindexer60(),
                                                new SleepAction(1),
                                                AutoActions.moveSpindexer120(),
                                                new SleepAction(1),
                                                AutoActions.moveSpindexer120()
                                        )
                                ),
                                driveToPreloadShoot,
                                AutoActions.shooterTurnOnClose()




                        ),
                        AutoActions.robotUpdate()
        ));
    }
}
