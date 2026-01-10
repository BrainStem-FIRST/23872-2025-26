package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.BetterRedFarAuto.collect123Pre;
import static org.firstinspires.ftc.teamcode.auto.BetterRedFarAuto.collect3;
import static org.firstinspires.ftc.teamcode.auto.BetterRedFarAuto.collect456Pre;
import static org.firstinspires.ftc.teamcode.auto.BetterRedFarAuto.collect6;
import static org.firstinspires.ftc.teamcode.auto.BetterRedFarAuto.collect9;
import static org.firstinspires.ftc.teamcode.utils.pidDrive.UtilFunctions.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;
@Disabled
@Autonomous(name="Closee")
@Config
public class RedClosest extends LinearOpMode {
    public static double[] start = new double[] { -63, 36, 0 };
    public static double[] close1Shooting = new double[] { -15, 22, 135 };

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
                new Waypoint(createPose(collect123Pre)),
                new Waypoint(createPose(collect3)).setMaxLinearPower(0.2)
        );
        DrivePath driveToCollectSecondSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect456Pre)),
                new Waypoint(createPose(collect6)).setMaxLinearPower(0.2)
        );
        DrivePath driveToCollectThirdSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect456Pre)),
                new Waypoint(createPose(collect9)).setMaxLinearPower(0.2)
        );

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                // shoot preloads
                                driveToPreloadShoot,
                                new SleepAction(2),

                                shootBalls(),

                                // collect first spike
                                AutoActions.setCollectorOn(),

                                new ParallelAction(
                                        driveToCollectFirstSpike,
                                        new SequentialAction(
                                                new SleepAction(1),
                                                AutoActions.moveSpindexer120(),
                                                new SleepAction(1),
                                                AutoActions.moveSpindexer120()
                                        )
                                ),

                                driveToPreloadShoot,

                                shootBalls(),

                                new ParallelAction(
                                        driveToCollectSecondSpike,
                                        new SequentialAction(
                                                new SleepAction(1),
                                                AutoActions.moveSpindexer120(),
                                                new SleepAction(1),
                                                AutoActions.moveSpindexer120()
                                        )
                                ),

                                driveToPreloadShoot,
                                shootBalls(),

                                new ParallelAction(
                                        driveToCollectThirdSpike,
                                        new SequentialAction(
                                                new SleepAction(1),
                                                AutoActions.moveSpindexer120(),
                                                new SleepAction(1),
                                                AutoActions.moveSpindexer120()
                                        )
                                ),

                                driveToPreloadShoot,

                                shootBalls()

                        ),
                        AutoActions.robotUpdate(telemetry)
                ));
    }

    private Action shootBalls() {
        return new SequentialAction(
                AutoActions.moveSpindexer60(),
                AutoActions.fingerServoU(),
                AutoActions.fingerServoD(),
                new SleepAction(0.2),
                AutoActions.moveSpindexer120(),
                new SleepAction(0.2),
                AutoActions.fingerServoU(),
                AutoActions.fingerServoD(),
                new SleepAction(0.2),
                AutoActions.moveSpindexer120(),
                new SleepAction(0.2),
                AutoActions.fingerServoU(),
                AutoActions.fingerServoD(),
                new SleepAction(0.2),
                AutoActions.moveSpindexer60()

        );
    }
}
