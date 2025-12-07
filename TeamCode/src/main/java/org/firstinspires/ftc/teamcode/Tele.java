package org.firstinspires.ftc.teamcode;



import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;

import tele_subsystems.Collector;
import tele_subsystems.Finger;
import tele_subsystems.Shooter;
import tele_subsystems.Spindexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ShootThreeBalls;


@TeleOp(name = "TeleOp")
public class Tele extends LinearOpMode {
    private GamepadTracker gp1;
    private GamepadTracker gp2;
    private ElapsedTime runtime = new ElapsedTime();
    private BrainSTEMTeleRobot robot;
    private ShootThreeBalls shootThreeBalls;

    private boolean aWasPressed;
    private boolean yWasPressed;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BrainSTEMTeleRobot(this.hardwareMap, this.telemetry, this, new Pose2d(0, 0, 0));
        shootThreeBalls = new ShootThreeBalls(this.robot.shooter, this.robot.finger, this.robot.spindexer, telemetry);

        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);



//        Shooter shooter;
//        Finger finger;
//        Spindexer spindexer;
//        Telemetry telemetry;

        waitForStart();

        while (!opModeIsActive()) {


            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            robot.update();
            gp1.update();;
            gp2.update();

            //Gamepad 1 controls ↓
            if (gp1.isFirstA()) {
                robot.collector.collectorState = Collector.CollectorState.ON;
            } else {
                robot.collector.collectorState = Collector.CollectorState.OFF;
            }

            if (gamepad1.b) {
                robot.shooter.shooterState = Shooter.ShooterState.SHOOT_FAR;
            } else {
                robot.shooter.shooterState = Shooter.ShooterState.OFF;
            }


            //Gamepad 2 controls ↓
            if (gamepad2.a) {
//                robot.spindexer.rotate120degrees();
                robot.spindexer.spindexerState = Spindexer.SpindexerState.NORMAL;
                robot.spindexer.rotate120degrees();
            }
//            } else if (gamepad2.y && !robot.spindexer.isSpindexerBusy()){
//                robot.spindexer.rotateDegrees(60);
//            }


            if (gamepad2.b) {
                robot.finger.fingerState = Finger.FingerState.UP;
            } else {
                robot.finger.fingerState = Finger.FingerState.DOWN;
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

            telemetry.addData("spindexer", robot.spindexer.spindexerMotor.getPower());


            telemetry.addData("y-axis :", y);
            telemetry.addData("x-axis :", x);
            telemetry.addData("turn :", rx);


        }
    }
}