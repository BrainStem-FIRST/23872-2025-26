package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.math.HeadingCorrect;

/*
What to do:

RETUNE COLOR SENSOR

AUTO ADJUSTING BUTTON THAT ADJUSTS IN THE RIGHT DIRECTION, WHEN CLOSE ENOUGH GP RUMBLEs

 ADD SMTH THAT COUNTS BALLS SO DONT HAVE TO RELY ON BALL TRACKER

INTAKE SLOWS WHEN A BALL IS TRYING TO BE TURNED

ADD ELApSED TIMER AFTER DETECTING BALL AND BEOFRE SHOOTING


AUTO PIVOTING CODE



P1: auto pivoting code for far shooting & anti jamming
P2: optimizing uato indexinga
P3: auto
 */
@TeleOp(name = "Potential Tele Yay")
public class PotentialTele extends LinearOpMode {
    private GamepadTracker gp1;
    private GamepadTracker gp2;
    private BrainSTEMRobot robot;

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

        alignmentPID = new PIDController(
                Constants.DriveConstants.ALIGNMENT_KP,
                Constants.DriveConstants.ALIGNMENT_KI,
                Constants.DriveConstants.ALIGNMENT_KD
        );

        // Switch goal
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

        telemetry.addLine("Robot is Ready!");
        telemetry.update();

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



        }
    }


    private void updateDriver1() {

        // DRIVING ==========================================
        double y = -gamepad1.left_stick_y * 0.99;
        double x = gamepad1.left_stick_x * 0.99;
        double rx = gamepad1.right_stick_x * 0.75;

//        if (gamepad1.y) {
//            double dx = goal.x - robot.drive.localizer.getPose().position.x;
//            double dy = goal.y - robot.drive.localizer.getPose().position.y;
//
//            double targetAngle = Math.atan2(dy, dx);
//            double currentHeading = robot.drive.localizer.getPose().heading.toDouble();
//            double error = Angle.normDelta(targetAngle - currentHeading);
////            double error = HeadingCorrect.correctHeadingErrorRad(targetAngle - currentHeading); TODO: IS THIS CORRECT???
//
//            alignmentPID.setTarget(targetAngle);
//
////            rx = alignmentPID.updateWithError(error); // TODO: TEST THIS
//            rx = alignmentPID.update(currentHeading);
//        }

        robot.drive.setMotorPowers(
                y + x + rx,
                y - x - rx,
                y - x + rx,
                y + x - rx
        );

        // SUBSYSTEM CONTROLS =====================================================
        if (gamepad1.right_trigger > 0.1 ) {
            robot.collector.collectorState = Collector.CollectorState.EXTAKE;
        } else if (gamepad1.right_bumper) {
            robot.collector.collectorState = Collector.CollectorState.INTAKE;
        } else {
            robot.collector.collectorState = Collector.CollectorState.OFF;
        }

        if (gp1.isFirstLeftBumper()) {
            robot.spindexer.setTargetAdj(Constants.spindexerConstants.TICKS_120);
        }


    }

    private void updateDriver2() {

        if (gamepad2.yWasPressed()) {
            robot.shooter.setShooterShootFar();
        } else if (gp2.isFirstA()) {
            robot.shooter.setShooterShootClose();
        } else if (gp2.isFirstB()) {
            robot.shooter.setShooterIdle();
        } else if (gp2.isFirstX()) {
            robot.shooter.setShooterOff();
        }



        if (gp2.isFirstLeftBumper()) {
            robot.pivot.setPivotShootClose();
        } else if (gp2.isFirstLeftTrigger()) {
            robot.pivot.setPivotShootFar();
        }

        if (gp2.isFirstDpadRight()) {
            robot.ramp.setRampUp();
        } else if (gp2.isFirstDpadLeft()) {
            robot.ramp.setRampDown();
        }

        if (gp2.isFirstDpadUp()) {
            robot.spindexer.setTargetAdj(341);
        }

        if (gp2.isFirstRightBumper()) {
            robot.spindexer.setTargetAdj(1024);
        }

//        if (gp2.isFirstDpadUp()) {
//            robot.spindexer.setTargetAdj(100);
//        }

    }
}