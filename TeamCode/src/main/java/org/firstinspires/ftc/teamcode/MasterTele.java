package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tele_subsystems.Collector;
import org.firstinspires.ftc.teamcode.tele_subsystems.Finger;
import org.firstinspires.ftc.teamcode.tele_subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.GamepadTracker;



@TeleOp(name = "Competition Tele")
public class MasterTele extends LinearOpMode {
    private GamepadTracker gp1;
    private GamepadTracker gp2;
    private ElapsedTime runtime = new ElapsedTime();
    private BrainSTEMTeleRobot robot;

    private ElapsedTime shoot3balls;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot = new BrainSTEMTeleRobot(hardwareMap, this.telemetry, this, new Pose2d(0, 0, 0));

        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);

        shoot3balls = new ElapsedTime();

        robot.shooter.setShooterIdle();

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
            telemetry.addData("shooter pid target", robot.shooter.shooterPID1.getTarget());

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
        double y = -gamepad1.left_stick_y * 0.99;
        double x = gamepad1.left_stick_x * 0.99;
        double rx = gamepad1.right_stick_x * 0.75;


        robot.drive.setMotorPowers(
                y + x + rx,
                y - x - rx,
                y - x + rx,
                y + x - rx

        );


        //d1 intake controls
        if (gamepad1.right_trigger > 0.1) {
            robot.collector.collectorState = Collector.CollectorState.INTAKE;
        } else if (gamepad1.left_trigger > 0.1) {
            robot.collector.collectorState = Collector.CollectorState.EXTAKE;
        } else {
            robot.collector.collectorState = Collector.CollectorState.OFF;
        }


        //d1 shooter controls
        if (gamepad1.yWasPressed()) {
            robot.shooter.setShooterShootFar();
        }
        if (gamepad1.bWasPressed()) {
            robot.shooter.setShooterShootClose();
        }
        if (gamepad1.aWasPressed()) {
            robot.shooter.setShooterIdle();
        }

        // d1 shooting controls
        if (gamepad1.xWasPressed()){
            robot.finger.fingerState = Finger.FingerState.UP;
            robot.spindexer.indexerCued = true;
            robot.finger.flickerTimer.reset();
        }

        //d1 spindexer controls
        if (gamepad1.leftBumperWasPressed()){
            robot.spindexer.adjustPosition(-80);
        } else if (gamepad1.rightBumperWasPressed()){
            robot.spindexer.adjustPosition(80);
        }

        if (gamepad1.dpadRightWasPressed()){
            robot.spindexer.adjustPosition(10);
        } else if (gamepad1.dpadLeftWasPressed()){
            robot.spindexer.adjustPosition(-10);
        }


    }

    private void updateDriver2() {

        robot.spindexer.getCurrentPosition();



        // d2 controls
        if (gamepad2.rightBumperWasPressed()) {
            robot.spindexer.adjustPosition(80);
            robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT;
        } else if (gamepad2.leftBumperWasPressed()) {
            robot.spindexer.adjustPosition(-80);
        } else if (gamepad2.aWasPressed() && robot.spindexer.spindexerState == Spindexer.SpindexerState.COLLECT) {
            robot.spindexer.adjustPosition(40);
            robot.spindexer.spindexerState = Spindexer.SpindexerState.SHOOT;
        } else if (gamepad2.xWasPressed() && robot.spindexer.spindexerState == Spindexer.SpindexerState.SHOOT) {
            robot.spindexer.adjustPosition(-80);
            robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT;
        }

        // d2 shoot 1 ball
        if (gamepad2.bWasPressed()) {
            robot.finger.fingerState = Finger.FingerState.UP;
            robot.spindexer.indexerCued = true;
            robot.finger.flickerTimer.reset();
            robot.spindexer.spindexerTimer.reset();
        }

        // d2 fine adjust
        if (gamepad2.left_trigger > 0.5){
            robot.spindexer.adjustPosition(-10);
        } else if (gamepad2.right_trigger > 0.5){
            robot.spindexer.adjustPosition(10);
        }

        //d2 shooter
        if (gamepad2.dpadUpWasPressed()){
            robot.shooter.setShooterOff();
        }
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
    private double autoAlignRoboOdo() {

        double angle = calculateAngle();
        double headingError = angle - robot.drive.localizer.getPose().heading.toDouble();


        while (headingError > Math.PI) {
            headingError -= 2 * Math.PI;
        }
        while (headingError < -Math.PI) {
            headingError += 2 * Math.PI;
        }
        if (Math.abs(headingError) <= Math.toRadians(3)){
            return 0;
        }

        telemetry.addData("error", headingError);
        double power = 0.05*headingError; //FIXME
        double minPower = 0.15;
        if (Math.abs(power) < minPower) {
            power = Math.copySign(minPower, power);
        }

        telemetry.addData("power", power);
        return -power;

    }
}