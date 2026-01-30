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

/*




TELEOP CONTROLS

D1 – DRIVE / ACQUIRE (hands stay on sticks)
LS                : Drive
RS                : Turn
RB (hold)         : Intake ON
RT (hold)         : Intake REV
LB                : Spindexer +60
LT                : Spindexer -60
Y (hold)          : Auto-align

D2 – SHOOT
Y                 : Shooter HIGH (tap)
A                 : Shooter LOW (tap)
B                 : Shooter IDLE (tap)
X                 : Shooter OFF (tap)
LB                : Ramp UP (tap)
LT                : Ramp DOWN (tap)
RB                : FIRE (tap)
D-Pad U / D        : Spindexer fine + / - (tap)

WIRING ========================================
Control:

Motors:
0: FL
1: BL
2: shooterMotorOne + encoder
3: spindexerMotor + external encoder

Servos:
0: leftPark
1:
2:
3:
4:
5: leftHood




Expansion:
0: FR
1: BR
2: shooterMotorTwo + encoder
3: collectorMotor

Servos:

0: rightHood
1:
2:
3:
4:
5: rightPark




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

        robot = new BrainSTEMRobot(hardwareMap, this.telemetry, this, new Pose2d(BrainSTEMRobot.autoX, BrainSTEMRobot.autoY, BrainSTEMRobot.autoH));

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

            telemetry.addLine("\n=== DRIVE ===");
            telemetry.addData("Pose", robot.drive.localizer.getPose().toString());
            telemetry.addData("Align Target", alignmentPID.getTarget());

            telemetry.addLine("\n=== SHOOTER ===");
            telemetry.addData("State", robot.shooter.shooterState);
            telemetry.addData("Vel", robot.shooter.shooterMotorOne.getVelocity());
            telemetry.addData("At Speed", Math.abs(robot.shooter.shooterMotorOne.getVelocity() - robot.shooter.targetVel) < 50);

            telemetry.addLine("\n=== SPINDEXER ===");
            telemetry.addData("State", robot.spindexer.spindexerState);
            telemetry.addData("Position", robot.spindexer.getCurrentPosition());
            telemetry.addData("Target", robot.spindexer.spindexerPid.getTarget());
            telemetry.addData("Current (mA)", robot.spindexer.spindexerMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("AntiJam Timer", robot.spindexer.antijamTimer);

            telemetry.addLine("\n=== SUBSYSTEMS ===");
            telemetry.addData("Collector", robot.collector.collectorState);
//            telemetry.addData("Ramp", robot.ramp.rampState);

//            telemetry.addData("R", robot.ballSensor.colorSensor.red());
//            telemetry.addData("G", robot.ballSensor.colorSensor.green());
//            telemetry.addData("B", robot.ballSensor.colorSensor.blue());
//            telemetry.addData("Alpha", robot.ballSensor.colorSensor.alpha());

            telemetry.addData("Detected Color", robot.ballSensor.detectColor());






            telemetry.update();

        }
    }


    private void updateDriver1() {

        // DRIVING ==========================================
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

        // SUBSYSTEM CONTROLS =====================================================
        if (gamepad1.right_trigger > 0.1 ) {
            robot.collector.collectorState = Collector.CollectorState.INTAKE;
        } else if (gamepad1.right_bumper) {
            robot.collector.collectorState = Collector.CollectorState.EXTAKE;
        } else {
            robot.collector.collectorState = Collector.CollectorState.OFF;
        }

        if (gp1.isFirstLeftBumper()) {
            robot.spindexer.SPINDEXER_TIME = 0;
            robot.spindexer.setSpindexerTargetAdjustment(Constants.SpindexerConstants.TICKS_120);
//            robot.limelight.ballTrackerNew.rotated60();
        } else if (gp1.isFirstLeftTrigger()) {
            robot.spindexer.SPINDEXER_TIME = 0;
            robot.spindexer.setSpindexerTargetAdjustment(-48);
//            robot.limelight.ballTrackerNew.rotatedNeg60();
        }

    }

    private void updateDriver2() {

        if (gp2.isFirstY()) {
            robot.shooter.setShooterShootFar();
        } else if (gp2.isFirstA()) {
            robot.shooter.setShooterShootClose();
        } else if (gp2.isFirstB()) {
            robot.shooter.setShooterIdle();
        } else if (gp2.isFirstX()) {
            robot.shooter.setShooterOff();
        }

        if (gp2.isFirstLeftBumper()) {
//            robot.ramp.setRampUp();
        } else if (gp2.isFirstLeftTrigger()) {
//            robot.ramp.setRampDown();
        }

        if (gp2.isFirstRightBumper()) {
            robot.spindexer.setSpindexerTargetAdjustment((robot.limelight.ballTrackerNew.getBestRotation()/Constants.SpindexerConstants.TICKS_360) * 8192);
        }

        if (gp2.isFirstDpadUp()) {
            robot.spindexer.setSpindexerTargetAdjustment(5);
        } else if (gp2.isFirstDpadDown()) {
            robot.spindexer.setSpindexerTargetAdjustment(-5);
        }

    }
}