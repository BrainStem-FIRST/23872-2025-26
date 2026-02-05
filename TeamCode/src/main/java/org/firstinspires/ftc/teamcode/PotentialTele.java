package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.PIDController;

/*
What to do:

detected color doesn't update when ball is take out - problem // when u take a ball out and skip from it, the ball sensors doesnt read the color
Fix logic for optimization to use closest one, and go from there
check color sensor after every spindexer trun, in case ball didnt shoot - doesnt work
Indexer not assigning colors to slot properly

 TUNE DELAY AND SETTLE TIMES

 MAKE DEFINATIVE ROCK SPINDEXER POS THAT ARENT ADDING OFF OF PREV TRARGET POSITIOJ
 D2 BE ABLE TO OVERRIDE THIS
 SMALL ADJUSTMENT TO WHAT U BELIEVE IS IN THE RIGHT DIRECTION

 MAKE DELAY TIMES SO THAT SPIND SPINS AS SOON AS IT CAN

 USE COLOR SENSRO TO DETECT WHEN ITS SOLIDLY IN THX KEERTHANA
 set velocity, spindexer turn and amp spike then power off for a sec

SPINDEXER PID NEEDS FIXING
 make it so that it so theres a color unknown - you spin

 TO TEST:
 BALL BUMP UP OTHER BALL TO SHOOTER

 Fixed:
 spindexer spin repeatedly, doesnt stop - FIXED Problem: wrong encoder wire
 spindexer has no resistence when turning -- not keeping target pos - fixed
 fix detect color so that it doesnt try to turn 120 when it sees the bad edges of spindles
shooter doesnt work FIXXXX - maybe motors are fighting each other - wrong run mode





// TODO: Tune color sensor vals


TELEOP CONTROLS

D1 – DRIVE / ACQUIRE
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
2: Ramp
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



        }
    }


    private void updateDriver1() {

        // DRIVING ==========================================
        double y = -gamepad1.left_stick_y * 0.99;
        double x = gamepad1.left_stick_x * 0.99;
        double rx = gamepad1.right_stick_x * 0.75;

        if (gamepad1.y) {
            double dx = goal.x - robot.drive.localizer.getPose().position.x;
            double dy = goal.y - robot.drive.localizer.getPose().position.x;

            double targetAngle = Math.atan2(dy, dx);
            double currentHeading = robot.drive.localizer.getPose().heading.toDouble();
            double angleError = targetAngle - currentHeading;

            while (angleError > Math.PI) angleError -= 2 * Math.PI;
            while (angleError < -Math.PI) angleError += 2 * Math.PI;
            rx = alignmentPID.update(angleError);
        }

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
//            robot.limelight.ballTrackerNew.rotated60();
        } else if (gp1.isFirstLeftTrigger()) {

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
//            robot.ramp.setRampUp();
        } else if (gp2.isFirstLeftTrigger()) {
            robot.pivot.setPivotShootFar();
        }

        if (gp2.isFirstDpadRight()) {
            robot.ramp.setRampUp();
        } else if (gp2.isFirstDpadLeft()) {
            robot.ramp.setRampDown();
        }

        if (gp2.isFirstRightBumper()) {
//            robot.spindexer.setTargetAdj((robot.limelight.ballTrackerNew.getBestRotation()/Constants.spindexerConstants.TICKS_360) * 8192);
        }

        if (gp2.isFirstDpadUp()) {
            robot.spindexer.setTargetAdj(100);
        }

    }
}