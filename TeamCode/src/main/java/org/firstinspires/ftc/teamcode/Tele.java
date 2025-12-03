package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import tele_subsystems.Collector;
import tele_subsystems.Finger;
import tele_subsystems.Shooter;
import tele_subsystems.Spindexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ShootThreeBalls;


@TeleOp(name = "TeleOp")
public class Tele extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private BrainSTEMTeleRobot robot;
    private ShootThreeBalls shootThreeBalls;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BrainSTEMTeleRobot(this.hardwareMap, this.telemetry, this, new Pose2d(0, 0, 0));
        shootThreeBalls = new ShootThreeBalls(this.robot.shooter, this.robot.finger, this.robot.spindexer, telemetry);

        Shooter shooter;
        Finger finger = null;
        Spindexer spindexer;
        Telemetry telemetry = null;

        waitForStart();

        while (!opModeIsActive()) {

            telemetry.addData("Opmode Status :", "Init");

            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            robot.update();

            //Gamepad 1 controls ↓
            if (gamepad1.a) {
                robot.collector.collectorState = Collector.CollectorState.ON;
            } else {
                robot.collector.collectorState = Collector.CollectorState.ON;
            }

            if (gamepad1.b) {
                robot.shooter.shooterState = Shooter.ShooterState.SHOOT_FAR;
            } else {
                robot.shooter.shooterState = Shooter.ShooterState.OFF;
            }

            //Gamepad 2 controls ↓
            if (gamepad2.a && !robot.spindexer.isSpindexerBusy()) {
                robot.spindexer.rotateDegrees(120);
            }

            if (gamepad2.b) {
                robot.finger.fingerState = Finger.FingerState.UP;
            } else {
                robot.finger.fingerState = Finger.FingerState.DOWN;
            }

            if (gamepad2.y && !robot.spindexer.isSpindexerBusy()) {
                robot.spindexer.rotateDegrees(60);
            }

            if (gamepad2.x && !robot.spindexer.isSpindexerBusy() && (finger.fingerState == Finger.FingerState.DOWN)) {
                shootThreeBalls.start();
            }

            //driving ↓
            double y = -gamepad1.left_stick_y * 0.6;
            double x = gamepad1.left_stick_x * 0.6;
            double rx = gamepad1.right_stick_x * 0.6;

            robot.drive.setMotorPowers(
                    y + x + rx,
                    y - x - rx,
                    y - x + rx,
                    y + x - rx
            );

            telemetry.addData("frontLeft", robot.drive.leftFront.getPower());
            telemetry.addData("frontRight", robot.drive.rightFront.getPower());
            telemetry.addData("backLeft", robot.drive.leftBack).getClass();
            telemetry.addData("backRight", robot.drive.rightBack).getClass();


            telemetry.addData("y-axis :", y);
            telemetry.addData("x-axis :", x);
            telemetry.addData("turn :", rx);


        }
    }
}