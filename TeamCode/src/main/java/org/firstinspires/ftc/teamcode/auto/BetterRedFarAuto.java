package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.pidDrive.MathUtils.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.pidDrive.Waypoint;

@Autonomous(name="BetterRedCloseAuto")
@Config
public class BetterRedFarAuto extends LinearOpMode {
    public static double[] start = new double[] { 60, 24, 0 };

    public static double[] collect123Pre = new double[] { -13, 10, 90 };
    public static double[] collect456Pre = new double[] { 12, 10, 90 };
    public static double[] collect789Pre = new double[] { 24, 10, 90 };

    public static double[] collect1 = new double[] { -13, 43, 90 };
    public static double[] collect2 = new double[] { -13, 48, 90 };
    public static double[] collect3 = new double[] { -13, 53, 90 };

    public static double[] collect4 = new double[] { 12, 32, 90 };
    public static double[] collect5 = new double[] { 12, 37, 90 };
    public static double[] collect6 = new double[] { 12, 41, 90 };

    public static double[] collect7 = new double[] { 36, 32, 90 };
    public static double[] collect8 = new double[] { 36, 37, 90 };
    public static double[] collect9 = new double[] { 36, 41, 90 };

    public static double[] farShooting = new double[] {57, 9 , 0}; //change

    public static double collectMaxPower = 0.3;
    BrainSTEMAutoRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20); // faster updates

        robot = new BrainSTEMAutoRobot(hardwareMap, telemetry, this, createPose(start));
        AutoActions.setRobot(robot);

        DrivePath driveToPreloadShoot = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(farShooting))
        );


        DrivePath driveToCollectFirstSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect123Pre)),
             new Waypoint(createPose(collect1)).setMaxLinearPower(collectMaxPower)
        );
        DrivePath driveToShootFar = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(farShooting))

        );


        DrivePath driveToCollectEight = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect8)).setMaxLinearPower(collectMaxPower)

                );
        DrivePath driveToCollectNine = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect9)).setMaxLinearPower(collectMaxPower)

                );
        DrivePath driveToCollectSix = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect6)).setMaxLinearPower(collectMaxPower)

        );
        DrivePath driveToCollectFive = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect5)).setMaxLinearPower(collectMaxPower)

        );


        DrivePath driveToCollectThree = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect3)).setMaxLinearPower(collectMaxPower)

        );
        DrivePath driveToCollectTwo = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect2)).setMaxLinearPower(collectMaxPower)

        );

        DrivePath driveToCollectSecondSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect456Pre)),
                new Waypoint(createPose(collect4)).setMaxLinearPower(collectMaxPower)


        );

        DrivePath driveToCollectThirdSpike = new DrivePath(robot.drive, telemetry,
                new Waypoint(createPose(collect789Pre)),
                new Waypoint(createPose(collect7)).setMaxLinearPower(collectMaxPower)


        );

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                // shoot preloads
                                driveToShootFar,
                                new SleepAction(2),

                                shootBalls(),


                                new ParallelAction(
                                       new ParallelAction(
                                               AutoActions.setCollectorOn(),
                                               AutoActions.moveSpindexer60()
                                       ),
                                        driveToCollectThirdSpike
                                ),

                                new SleepAction(0.3),
                                AutoActions.moveSpindexer120(),
                                driveToCollectEight,
                                new SleepAction(0.3),
                                AutoActions.moveSpindexer120(),
                                driveToCollectNine,
                                new SleepAction(0.3),
                                new ParallelAction(
                                        driveToPreloadShoot,
                                        AutoActions.setCollectorOff(),
                                        AutoActions.shooterTurnOnFar()
                                ),

                                shootBalls(),

                                new SleepAction(0.3),


                                new ParallelAction(
                                        new ParallelAction(
                                                AutoActions.setCollectorOn(),
                                                AutoActions.moveSpindexer60()
                                        ),
                                        driveToCollectSecondSpike
                                ),
                                new SleepAction(0.3),
                                AutoActions.moveSpindexer120(),
                                driveToCollectFive  ,
                                new SleepAction(0.3),
                                AutoActions.moveSpindexer120(),
                                driveToCollectSix,
                                new SleepAction(0.3),
                                new ParallelAction(
                                        driveToPreloadShoot,
                                        AutoActions.setCollectorOff(),
                                        AutoActions.shooterTurnOnFar()
                                ),

                                new SleepAction(0.3),

                                shootBalls(),

                                new  SleepAction(0.3),


                                new ParallelAction(
                                        new ParallelAction(
                                                AutoActions.setCollectorOn(),
                                                AutoActions.moveSpindexer60()
                                        ),
                                        driveToCollectFirstSpike
                                ),

                                new SleepAction(0.3),
                                AutoActions.moveSpindexer120(),
                                driveToCollectTwo,
                                new SleepAction(0.3),
                                AutoActions.moveSpindexer120(),
                                driveToCollectThree,
                                new SleepAction(0.3),
                                new ParallelAction(
                                        driveToPreloadShoot,
                                        AutoActions.setCollectorOff(),
                                        AutoActions.shooterTurnOnFar()
                                ),

                                new SleepAction(0.3),

                                shootBalls()
                        ),
                        AutoActions.robotUpdate()
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