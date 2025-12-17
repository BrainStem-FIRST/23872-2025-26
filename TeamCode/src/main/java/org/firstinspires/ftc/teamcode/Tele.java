package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
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


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot = new BrainSTEMTeleRobot(this.hardwareMap, this.telemetry, this, new Pose2d(0, 0, 0));
        shootThreeBalls = new ShootThreeBalls(this.robot.shooter, this.robot.finger, this.robot.spindexer, telemetry);

        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);


//

        waitForStart();

        while (!opModeIsActive()) {


            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            robot.update();
            gp1.update();
            gp2.update();

            updateDriver1();
            updateDriver2();

            telemetry.addData("shooter power 1", robot.shooter.shooterMotorOne.getPower());
            telemetry.addData("shooter vel 1", robot.shooter.shooterMotorOne.getVelocity());
            telemetry.addData("shooter pid target", robot.shooter.shooterPid.getTarget());

            telemetry.addData("spindexer pos", robot.spindexer.getMotorPos());
            telemetry.addData("spindexer target position", robot.spindexer.spindexerPid.getTarget());
            telemetry.addData("spindexer state", robot.spindexer.spindexerState);
            telemetry.addData("indexer cued", robot.spindexer.indexerCued);
            telemetry.addData("finger timer", robot.finger.flickerTimer.seconds());

            telemetry.update();


        }
    }

    private void updateDriver1() {
        // if statements checking for d1 controls
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


        //Gamepad 1 controls ↓
        if (gamepad1.right_trigger > 0.1) {
            robot.collector.collectorState = Collector.CollectorState.ON;
        } else {
            robot.collector.collectorState = Collector.CollectorState.OFF;
        }

        if (gamepad1.yWasPressed()) {
            robot.shooter.shooterState = Shooter.ShooterState.SHOOT_FAR;
            robot.shooter.shooterPid.reset();
            robot.shooter.shooterPid.setTarget(Shooter.FAR_SHOOT_VEL);
        } else if (gamepad1.bWasPressed()) {
            robot.shooter.shooterState = Shooter.ShooterState.SHOOT_CLOSE;
            robot.shooter.shooterPid.reset();
            robot.shooter.shooterPid.setTarget(Shooter.CLOSE_SHOOT_VEL);
        }
        else if (gamepad1.aWasPressed())
            robot.shooter.shooterState = Shooter.ShooterState.OFF;
    }
    private void updateDriver2() {
        // all d2 commands
        if(gamepad2.rightBumperWasPressed()) {
            robot.spindexer.rotateDegrees(Spindexer.normalRotateDeg);
        }
        else if(gamepad2.leftBumperWasPressed()) {
            robot.spindexer.rotateDegrees(-Spindexer.normalRotateDeg);
        }
        else if(gamepad2.aWasPressed() && robot.spindexer.spindexerState == Spindexer.SpindexerState.COLLECT) {
            robot.spindexer.rotateDegrees(Spindexer.shootRotateDeg);
            robot.spindexer.spindexerState = Spindexer.SpindexerState.SHOOT;
        }
        else if(gamepad2.xWasPressed() && robot.spindexer.spindexerState == Spindexer.SpindexerState.SHOOT) {
            robot.spindexer.rotateDegrees(-Spindexer.shootRotateDeg);
            robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT;
        }


        if (gamepad2.bWasPressed()) {
            robot.finger.fingerState = Finger.FingerState.UP;
            robot.spindexer.indexerCued = true;
            robot.finger.flickerTimer.reset();
        }
    }
}
