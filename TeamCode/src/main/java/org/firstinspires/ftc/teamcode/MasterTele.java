package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.PIDController;


@TeleOp(name = "Competition Tele")
public class MasterTele extends LinearOpMode {
    private GamepadTracker gp1;
    private GamepadTracker gp2;
    private BrainSTEMRobot robot;

    private PIDController alignmentPID;

    Vector2d goal = new Vector2d(-72, 72); //default: red
    private boolean red = true;

//    public SequentialAction ShootingSequence() {
//        return new SequentialAction(
//                AutoActions.fingerServoU(),
//                new SleepAction(0.4),
//                AutoActions.moveSpindexer120(),
//                new SleepAction(0.3),
//                AutoActions.fingerServoU(),
//                new SleepAction(0.4),
//                AutoActions.moveSpindexer120(),
//                new SleepAction(0.3),
//                AutoActions.fingerServoU(),
//                new SleepAction(0.4),
//                AutoActions.moveSpindexer60(),
//                AutoActions.turnShooterOnIdle()
//        );
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot = new BrainSTEMRobot(hardwareMap, this.telemetry, this, new Pose2d(BrainSTEMRobot.autoX, BrainSTEMRobot.autoY, BrainSTEMRobot.autoH));

        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);


        robot.shooter.setShooterOff();

        alignmentPID = new PIDController(
                Constants.DriveConstants.ALIGNMENT_KP,
                Constants.DriveConstants.ALIGNMENT_KI,
                Constants.DriveConstants.ALIGNMENT_KD
        );

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


            telemetry.addLine("\n=== EMERGENCY DIAGNOSING ===");
            if (robot.spindexer.antijamTimer.milliseconds() < 500) {
                telemetry.addLine("STATUS: Spindexer Jammed");
            } else {
                telemetry.addLine("STATUS: No Jam");
            }

            
            telemetry.addLine("\n=== DRIVE ===");
            telemetry.addData("Pose", robot.drive.localizer.getPose().toString());
            telemetry.addData("Align Target", alignmentPID.getTarget());

            telemetry.addLine("\n=== SHOOTER ===");
            telemetry.addData("State", robot.shooter.shooterState);
            telemetry.addData("Vel", robot.shooter.shooterMotorOne.getVelocity());
//            telemetry.addData("At Speed", Math.abs(robot.shooter.shooterMotorOne.getVelocity() - robot.shooter.get.getTarget()) < 50);
            // TODO: get error with internal PID


            telemetry.addLine("\n=== SPINDEXER ===");

            telemetry.addData("Position", robot.spindexer.getCurrentPosition());
            telemetry.addData("Target", robot.spindexer.spindexerPid.getTarget());
            telemetry.addData("Current (mA)", robot.spindexer.spindexerMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("AntiJam Timer", robot.spindexer.antijamTimer);



            telemetry.addLine("\n=== SUBSYSTEMS ===");
            telemetry.addData("Collector", robot.collector.collectorState);
//            telemetry.addData("Finger", robot.finger.fingerState);
//            telemetry.addData("Flicker Timer", robot.finger.flickerTimer.seconds());

            telemetry.update();

        }
    }


    private void updateDriver1() {
        // if statements checking for d1 controls
        //driving ↓
        double y = -gamepad1.left_stick_y * 0.99;
        double x = gamepad1.left_stick_x * 0.99;
        double rx = gamepad1.right_stick_x * 0.75;

        if (gamepad1.y) {
            double targetAngle = Math.atan2(goal.y, goal.x);
            double currentHeading = robot.drive.localizer.getPose().heading.toDouble();


            alignmentPID.setTarget(targetAngle);
            rx = alignmentPID.update(currentHeading);

        }

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

    }

    private void updateDriver2() {
        // d2 controls
        if (gamepad2.rightBumperWasPressed() || gamepad1.rightBumperWasPressed()) {
            robot.spindexer.SPINDEXER_TIME = 500;
            robot.spindexer.setTargetAdj(48);
//            robot.limelight.ballTrackerNew.rotated60();
        } else if (gamepad2.leftBumperWasPressed() || gamepad1.leftBumperWasPressed()) {
            robot.spindexer.SPINDEXER_TIME = 500;
            robot.spindexer.setTargetAdj(-48);
//            robot.limelight.ballTrackerNew.rotatedNeg60();
        }

        if (gamepad2.xWasPressed()) {
//            robot.finger.fingerState = Finger.FingerState.UP;
//            robot.finger.flickerTimer.reset();
//            robot.ramp.setRampUp();
        }

//        if (gamepad2.aWasPressed() && currentAction == null) {
//            currentAction = ShootingSequence();
//        }
//
//        if (currentAction != null) {
//            TelemetryPacket packet = new TelemetryPacket();
//            boolean keepGoing = currentAction.run(packet);
//            if (!keepGoing)
//                currentAction = null;
//        }

        if (gamepad2.yWasPressed()) {
            robot.shooter.setShooterShootFar();
        }

        if (gamepad2.bWasPressed()) {
            robot.shooter.setShooterShootClose();
        }

//        if (gamepad2.aWasPressed()) {
//            robot.spindexer.setSpindexerTargetAdjustment(robot.limelight.ballTracker.getBestRotation());
//            robot.ramp.setRampUp();
//            robot.spindexer.setSpindexerTargetAdjustment(Constants.spindexerConstants.TICKS_360);
//        }


        if (gamepad1.dpadUpWasPressed()){
//            robot.finger.fingerState = Finger.FingerState.UP;
//            robot.finger.flickerTimer.reset();
//            robot.ramp.setRampUp();
        } else if (gamepad1.dpadDownWasPressed()){
//            robot.finger.fingerState = Finger.FingerState.DOWN;
//            robot.ramp.setRampUp();
        }
        if (gp2.isFirstLeftTrigger()){
            robot.spindexer.SPINDEXER_TIME = 0;
            robot.spindexer.setTargetAdj(3);
        } else if (gp2.isFirstRightTrigger()){
            robot.spindexer.SPINDEXER_TIME = 0;
            robot.spindexer.setTargetAdj(-3);
        }

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
        return angle;
    }
}