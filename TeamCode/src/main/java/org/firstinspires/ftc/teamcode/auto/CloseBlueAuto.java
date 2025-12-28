package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.firstinspires.ftc.teamcode.auto_subsystems.Shooter;
import org.firstinspires.ftc.teamcode.auto_subsystems.Spindexer;

@Autonomous (name = "Blue Close Auto")
@Config
public final class CloseBlueAuto extends LinearOpMode {


    Coordinates coordinates;



    @Override
    public void runOpMode() throws InterruptedException {

        coordinates = new Coordinates(false); // making a local variable of coordinate and setting the red variable to false bc this is a red auto

        BrainSTEMAutoRobot robot = new BrainSTEMAutoRobot(hardwareMap, telemetry, this, coordinates.getCloseStartPose());


        Action robotUpdate = new AutoActions().robotUpdate(robot);

        Action moveSpindexer120 = new AutoActions().moveSpindexer120(robot);

        Action moveSpindexer60 = new AutoActions().moveSpindexer60(robot);

        Action shooterTurnOnFar = new AutoActions().shooterTurnOnFar(robot);

        Action shooterTurnOff = new AutoActions().shooterTurnOff(robot);

        Action fingerServoU = new AutoActions().fingerServoU(robot);

        Action fingerServoD = new AutoActions().fingerServoD(robot);

        Action setCollectorOn = new AutoActions().setCollectorOn(robot);

        Action setCollectorOff = new AutoActions().setCollectorOff(robot);


        Action collect1Drive = robot.drive.actionBuilder(coordinates.getCloseStartPose())
                .splineToLinearHeading(coordinates.getCloseCollect3ballsPrePose(), coordinates.getCloseCollect3BallsDriveTangent())
                .build();


        Action collect1stBall = robot.drive.actionBuilder(coordinates.getCloseCollect3ballsPrePose())
                .splineToLinearHeading(coordinates.getCloseCollect1BallPose(), coordinates.getCollectTangenet())
                .build();

        Action collect2ndBall = robot.drive.actionBuilder(coordinates.getCloseCollect1BallPose())
                .splineToLinearHeading(coordinates.getCloseCollect2BallPose(), coordinates.getCollectTangenet())
                .build();

        Action collect3rdBall = robot.drive.actionBuilder(coordinates.getCloseCollect2BallPose())
                .splineToLinearHeading(coordinates.getCloseCollect3BallPose(), coordinates.getCollectTangenet())
                .build();

        Action closeShootDrive = robot.drive.actionBuilder(coordinates.getCloseCollect3BallPose())
//                .splineToLinearHeading(coordinates.getCloseShootingPose(), coordinates.getCloseShootDriveTangent())
                .build();





        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        new Action[]{new SequentialAction(

                                // shoot the 3 preload balls
                                shooterTurnOnFar, // will need to change to shoot less
                                new SleepAction(1),
                                fingerServoU,
                                new SleepAction(1),
                                fingerServoD,
                                new SleepAction(1),
                                moveSpindexer120,
                                new SleepAction(1),
                                fingerServoU,
                                new SleepAction(1),
                                fingerServoD,
                                new SleepAction(1),
                                moveSpindexer120,
                                new SleepAction(1),
                                fingerServoU,
                                new SleepAction(1),
                                fingerServoD,
                                shooterTurnOff,

                                // drive to collect the next 3 balls & collecting
                                setCollectorOn,
                                collect1Drive,
                                moveSpindexer60,
                                collect1stBall,
                                new SleepAction(1),
                                moveSpindexer120,
                                collect2ndBall,
                                new SleepAction(1),
                                moveSpindexer120,
                                collect3rdBall,
                                new SleepAction(1),
                                moveSpindexer60, // set spindexer back to shoot
                                setCollectorOff,

                                // drive to go shoot the 3 balls that were just collected
                                shooterTurnOnFar,
                                closeShootDrive,
                                new SleepAction(1),
                                fingerServoU,
                                new SleepAction(1),
                                fingerServoD,
                                new SleepAction(1),
                                moveSpindexer120,
                                new SleepAction(1),
                                fingerServoU,
                                new SleepAction(1),
                                fingerServoD,
                                new SleepAction(1),
                                moveSpindexer120,
                                new SleepAction(1),
                                fingerServoU,
                                new SleepAction(1),
                                fingerServoD,
                                shooterTurnOff









                        ),
                                robotUpdate})

        );





        robot.update();


        while (opModeIsActive());





    }



}
