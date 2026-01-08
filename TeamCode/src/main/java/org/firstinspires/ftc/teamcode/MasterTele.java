package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.subsystems.Finger;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.GamepadTracker;
import org.firstinspires.ftc.teamcode.util.PIDController;


@TeleOp(name = "Competition Tele")
public class MasterTele extends LinearOpMode {
    private GamepadTracker gp1;
    private GamepadTracker gp2;
    private ElapsedTime runtime = new ElapsedTime();
    private BrainSTEMRobot robot;

    public static double kP = 1.1, kI = 0.13, kD = 0;

    private PIDController alignmentPID;

    Vector2d goal = new Vector2d(-72, 72); //default: red
    private boolean red = true;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot = new BrainSTEMRobot(hardwareMap, this.telemetry, this, new Pose2d(0, 0, 0));

        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);


        robot.shooter.setShooterOff();

        alignmentPID = new PIDController(kP, kI, kD);

        if (gamepad1.x){
            goal = new Vector2d(-72, -72);
            red = false;
        } else if (gamepad1.a){
            red = true;
        }

        if (red){
            telemetry.addLine("Color is Red");
        } else {
            telemetry.addLine("Color is Blue");
        }

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

//            telemetry.addData("shooter power 1", robot.shooter.shooterMotorOne.getPower());
//            telemetry.addData("shootr power 2", robot.shooter.shooterMotorTwo.getPower());
//            telemetry.addData("shooter vel 1", robot.shooter.shooterMotorOne.getVelocity());
//            telemetry.addData("shooter pid target", robot.shooter.shooterPID.getTarget());
//
//            telemetry.addData("spindexer pos", robot.spindexer.getMotorPos());
//            telemetry.addData("spindexer target position", robot.spindexer.spindexerPid.getTarget());
//            telemetry.addData("spindexer state", robot.spindexer.spindexerState);
////            telemetry.addData("indexer cued", robot.spindexer.indexerCued);
//
//            telemetry.addData("finger timer", robot.finger.flickerTimer.seconds());
//
//            telemetry.update();

            telemetry.addLine("\n=== EMERGENCY DIAGNOSING ===");
            if (robot.spindexer.antijamTimer.milliseconds() < 500) {
                telemetry.addLine("STATUS: Spindexer Jammed");
            } else {
                telemetry.addLine("STATUS: No Jam");
            }

            if (robot.shooter.shooterPID.getTarget() > 1000 && robot.shooter.avgMotorVel < 500) {
                telemetry.addLine("\n SHOOTER JAMMED OR UNPLUGGED");
                telemetry.addData("Target", robot.shooter.shooterPID.getTarget());
                telemetry.addData("Actual", robot.shooter.avgMotorVel);
            }
            
            telemetry.addLine("\n=== DRIVE ===");
            telemetry.addData("Pose", robot.drive.localizer.getPose().toString());
            telemetry.addData("Align Target", alignmentPID.getTarget());

            telemetry.addLine("\n=== SHOOTER ===");
            telemetry.addData("State", robot.shooter.shooterState);
            telemetry.addData("Avg Vel", robot.shooter.avgMotorVel); // From Shooter.java
            telemetry.addData("Target Vel", robot.shooter.shooterPID.getTarget());
            telemetry.addData("At Speed", Math.abs(robot.shooter.avgMotorVel - robot.shooter.shooterPID.getTarget()) < 50);
            telemetry.addData("Current (mA)", robot.spindexer.spindexerMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("AntiJam Timer", robot.spindexer.antijamTimer);


            telemetry.addLine("\n=== SPINDEXER ===");
            telemetry.addData("State", robot.spindexer.spindexerState);
            telemetry.addData("Position", robot.spindexer.getCurrentPosition());
            telemetry.addData("Target", robot.spindexer.spindexerPid.getTarget());



            telemetry.addLine("\n=== SUBSYSTEMS ===");
            telemetry.addData("Collector", robot.collector.collectorState);
            telemetry.addData("Finger", robot.finger.fingerState);
            telemetry.addData("Flicker Timer", robot.finger.flickerTimer.seconds());

            telemetry.update();

        }
    }


    private void updateDriver1() {
        // if statements checking for d1 controls
        //driving ↓
        double y = -gamepad1.left_stick_y * 0.99;
        double x = gamepad1.left_stick_x * 0.99;
        double rx = gamepad1.right_stick_x * 0.75;
//
//        if (gamepad2.y) {
////            rx = autoAlignRoboOdo();
//            double targetAngle = Math.atan2(goal.y, goal.x);
//            double currentHeading = robot.drive.localizer.getPose().heading.toDouble();
//
//
//            alignmentPID.setTarget(targetAngle);
//            rx = alignmentPID.update(currentHeading);
//
////            telemetry.addData("current target angle", targetAngle);
////            telemetry.addData("current heading angle", currentHeading);
//
//        }



        robot.drive.setMotorPowers(
                y + x + rx,
                y - x - rx,
                y - x + rx,
                y + x - rx

        );


        //d1 intake controls
        if (gamepad1.right_trigger > 0.1 ) {
            robot.collector.collectorState = Collector.CollectorState.INTAKE;
        } else if (gamepad1.left_trigger > 0.1) {
            robot.collector.collectorState = Collector.CollectorState.EXTAKE;
        } else {
            robot.collector.collectorState = Collector.CollectorState.OFF;
        }


        //d1 shooter controls
        if (gamepad2.yWasPressed()) {
            robot.shooter.setShooterShootFar();
        }
        if (gamepad2.bWasPressed()) {
            robot.shooter.setShooterShootClose();
        }
        if (gamepad2.aWasPressed()) {
            robot.shooter.setShooterIdle();
        }

        // d1 shooting controls
        if (gamepad2.xWasPressed()) {
            robot.finger.fingerState = Finger.FingerState.UP;
//            robot.spindexer.indexerCued = true;
            robot.finger.flickerTimer.reset();
        }
    }

    private void updateDriver2() {
        // d2 controls
        if (gamepad2.rightBumperWasPressed() || gamepad1.rightBumperWasPressed()) {
            robot.spindexer.setSpindexerTargetAdjustment(48);
        } else if (gamepad2.leftBumperWasPressed() || gamepad1.leftBumperWasPressed()) {
            robot.spindexer.setSpindexerTargetAdjustment(-48);
        }
//        } else if (gamepad2.aWasPressed() && robot.spindexer.spindexerState == Spindexer.SpindexerState.COLLECT) {
//            robot.spindexer.setSpindexerTargetAdjustment(40);
//            robot.spindexer.spindexerState = Spindexer.SpindexerState.SHOOT;
//        } else if (gamepad2.xWasPressed() && robot.spindexer.spindexerState == Spindexer.SpindexerState.SHOOT) {
//            robot.spindexer.setSpindexerTargetAdjustment(-40);
//            robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT;
//        }



        // d2 shoot 1 ball
        if (gamepad2.bWasPressed()) {
            robot.finger.fingerState = Finger.FingerState.UP;
            robot.spindexer.indexerCued = true;
            robot.finger.flickerTimer.reset();
//            robot.spindexer.spindexerTimer.reset();
        }

        if (gamepad1.dpadUpWasPressed()){
            robot.finger.fingerState = Finger.FingerState.UP;

            robot.finger.flickerTimer.reset();
        } else if (gamepad1.dpadDownWasPressed()){
            robot.finger.fingerState = Finger.FingerState.DOWN;
        }
        if (gp2.isFirstLeftTrigger()){
            robot.spindexer.setSpindexerTargetAdjustment(2);
        } else if (gp2.isFirstRightTrigger()){
            robot.spindexer.setSpindexerTargetAdjustment(-2);
        }

        //encoder ticks
//
            //d2 shooter
        if (gamepad2.dpadUpWasPressed()) {
            robot.shooter.setShooterOff();
        }
    }

    private double calculateAngle() {
        Vector2d redGoal = new Vector2d(-72, 72);
        Math.atan2(redGoal.x, redGoal.y);
        double dx = redGoal.x - robot.drive.localizer.getPose().position.x;
        double dy = redGoal.y - robot.drive.localizer.getPose().position.y;

        double angle = Math.atan2(dy, dx);
//        telemetry.addData("dx", dx);
//        telemetry.addData("dy", dy);
//        telemetry.addData("angle", angle);
        return angle;
    }
//    private double autoAlignRoboOdo() {
//
//        double angle = calculateAngle();
//        double headingError = angle - robot.drive.localizer.getPose().heading.toDouble();
//
//
//        while (headingError > Math.PI) {
//            headingError -= 2 * Math.PI;
//        }
//        while (headingError < -Math.PI) {
//            headingError += 2 * Math.PI;
//        }
//        if (Math.abs(headingError) <= Math.toRadians(3)) {
//            return 0;
//        }
//
//        telemetry.addData("error", headingError);
////        double power = alignP * headingError; //FIXME
//        double power = 0;
//        double minPower = 0.15;
//        if (Math.abs(power) < minPower) {
//            power = Math.copySign(minPower, power);
//        }
//
//        telemetry.addData("power", power);
//        return -power;
//
//    }
}