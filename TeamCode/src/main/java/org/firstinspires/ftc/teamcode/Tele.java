package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import tele_subsystems.Finger;
import tele_subsystems.Shooter;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class Tele extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private BrainSTEMTeleRobot robot;

    private boolean a_Button_Was_Pressed_Last_Loop = false;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BrainSTEMTeleRobot(this.hardwareMap, this.telemetry, this, new Pose2d(0, 0, 0));


        waitForStart();


        while (!opModeIsActive()) {

            telemetry.addData("Opmode Status :", "Init");

            telemetry.update();
        }


        while (opModeIsActive()) {
            telemetry.update();

            robot.update();

            boolean a_Button_Is_Pressed_This_Loop = gamepad2.x;
            if (a_Button_Is_Pressed_This_Loop && !a_Button_Was_Pressed_Last_Loop) {
                robot.spindexer.rotateDegrees(120);
            }
            a_Button_Was_Pressed_Last_Loop = a_Button_Is_Pressed_This_Loop;

            if (gamepad2.y && !robot.spindexer.isSpindexerBusy()) {
                robot.spindexer.rotateDegrees(60);
            }

            a_Button_Was_Pressed_Last_Loop = a_Button_Is_Pressed_This_Loop;

            if (gamepad1.a) {
//                robot.collector.collectorState = Collector.CollectorState.ON;
                robot.collector.collectorMotor.setPower(0.8);
            } else {
//                robot.collector.collectorState = Collector.CollectorState.ON;
                robot.collector.collectorMotor.setPower(0);
            }

            if (gamepad2.b) {
                robot.finger.fingerState = Finger.FingerState.UP;
            } else {
                robot.finger.fingerState = Finger.FingerState.DOWN;
            }


            if (gamepad1.b) {
                robot.shooter.shooterState = Shooter.ShooterState.SHOOT_FAR;
            } else {
                robot.shooter.shooterState = Shooter.ShooterState.OFF;
            }


            double y = -gamepad1.left_stick_y * 0.6;
            double x = gamepad1.left_stick_x * 0.6;
            double rx = gamepad1.right_stick_x * 0.6;

//            robot.drive.setMotorPowers(0.5, -0.5, -0.5,0.5);

            robot.drive.setMotorPowers(
                    y + x + rx,
                    y - x - rx,
                    y - x + rx,
                    y + x - rx
            );
//            telemetry.addData("frontLeft", robot.frontLeft.getPower());
//            telemetry.addData("frontRight", robot.frontRight.getPower());
//            telemetry.addData("backLeft", robot.backLeft.getPower());
//            telemetry.addData("backRight", robot.backRight.getPower());


            telemetry.addData("y-axis :", y);
            telemetry.addData("x-axis :", x);
            telemetry.addData("turn :", rx);
            //telemetry.update();


        }
    }
}