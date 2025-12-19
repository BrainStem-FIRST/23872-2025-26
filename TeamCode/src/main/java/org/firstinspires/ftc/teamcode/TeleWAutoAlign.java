package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;



import org.firstinspires.ftc.teamcode.tele_subsystems.Collector;
import org.firstinspires.ftc.teamcode.tele_subsystems.Finger;
import org.firstinspires.ftc.teamcode.tele_subsystems.Shooter;
import org.firstinspires.ftc.teamcode.tele_subsystems.Spindexer;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@TeleOp(name = "TeleOp")
@Config
public class TeleWAutoAlign extends LinearOpMode {
    private  GamepadTracker gp1;
    private GamepadTracker gp2;
    private ElapsedTime runtime = new ElapsedTime();
    private BrainSTEMTeleRobot robot;
    private ShootThreeBalls shootThreeBalls;

    public static double kP = 0.1;
    public static double DRAWING_TARGET_RADIUS = 2;

    enum Mode{
        DRIVER_CONTROL,
        ALIGN_TO_POINT
    }

    private Mode currentMode = Mode.DRIVER_CONTROL;

    private PIDController headingController = new PIDController(0,0,0); //change

    private Vector2d targetPosition = new Vector2d(0,0); //change

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot = new BrainSTEMTeleRobot(this.hardwareMap, this.telemetry, this, new Pose2d(0, 0, 0));
        shootThreeBalls = new ShootThreeBalls(this.robot.shooter, this.robot.finger, this.robot.spindexer, telemetry);

        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);



        //somehow transfer pose from auto

        headingController.setInputBounds(-Math.PI, Math.PI);

        waitForStart();

        while (!opModeIsActive()) {


            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            robot.update();
            gp1.update();
            gp2.update();

            Pose2d startingPose = new Pose2d(0,0,0); //change]

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
            telemetry.addData("mode",currentMode);

            telemetry.addData("current pose", robot.drive.localizer.getPose());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();

            Canvas fieldOverlay = packet.fieldOverlay();

            //put in function




        }
    }

    private void updateDriver1() {
        // if statements checking for d1 controls
        //driving ↓
        double y = -gamepad1.left_stick_y * 0.6;
        double x = gamepad1.left_stick_x * 0.6;
        double rx = gamepad1.right_stick_x * 0.6;

        if(gamepad2.y){
            rx = autoAlignRobo();
        }

        robot.drive.setMotorPowers(
                y + x + rx,
                y - x - rx,
                y - x + rx,
                y + x - rx
                
        );


        //Gamepad 1 controls ↓
        if (gamepad1.right_trigger > 0.1) {
            robot.collector.collectorState = Collector.CollectorState.INTAKE;
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

    public void setDriveModes() {
        switch(currentMode){
            case DRIVER_CONTROL:
                if (gamepad2.b){
                    currentMode = Mode.ALIGN_TO_POINT;
                }

                Vector2d fieldFrameInput = new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                );

                Vector2d robotFrameInput = rotateVector(fieldFrameInput, -robot.drive.localizer.getPose().heading.toDouble());
                Vector2d difference = targetPosition.minus(robot.drive.localizer.getPose().position);

                double theta = Math.atan2(difference.y, difference.x);



                break;

            case ALIGN_TO_POINT:
                break;
        }
    }
    private Vector2d rotateVector(Vector2d v, double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector2d(
                v.x * cos - v.y * sin,
                v.x * sin + v.y * cos
        );
    }


    private double calculateAngle(){
        Vector2d redGoal = new Vector2d(-72, 72);
        double dx = redGoal.x - robot.drive.localizer.getPose().position.x;
        double dy = redGoal.y - robot.drive.localizer.getPose().position.y;

        double angle = Math.atan2(dy,dx);
        telemetry.addData("dx", dx);
        telemetry.addData("dy", dy);
        telemetry.addData("angle", angle);
        return angle;
    }

    private double autoAlignRobo(){
        double angle = calculateAngle();
        double headingError = angle - robot.drive.localizer.getPose().heading.toDouble();

        double power = kP*headingError;
        telemetry.addData("power", power);
        return power;
    }

}
