package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.srs.SRSHub;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.subsystems.OneWShooter;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
//import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.sensors.Limelight;
import org.firstinspires.ftc.teamcode.utils.BallSensor;
import org.firstinspires.ftc.teamcode.utils.BallTrackerNew;
import org.firstinspires.ftc.teamcode.utils.Component;

public class BrainSTEMRobot {

    //NEEDS TO CHOOSE ONE
    // TODO: Clean up for new subsystems

    public static double autoX, autoY, autoH;
    // Don't touch these
    public Telemetry telemetry;
    public OpMode opMode;
    private final ArrayList<Component> subsystems;
    public Spindexer spindexer;
    public Collector collector;
    public MecanumDrive drive;

    public Limelight limelight;
    public BallSensor ballSensor;
    public Pivot pivot;
    public Ramp ramp;
    public OneWShooter shooter;
    public BallTrackerNew ballTracker;
    private boolean goodToMove = false;
    private BallTrackerNew.BallColor detectedColor;
    private ElapsedTime ballDetectTimer = new ElapsedTime();

    public boolean isSpindStopped;

    public boolean isNextEmpty;


    private boolean checkingColorAfterMovingSpind = false;

    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, Pose2d startPose) {

        this.telemetry = telemetry;
        this.opMode = opMode;

        ballSensor = new BallSensor(hwMap);
        subsystems = new ArrayList<>();

        spindexer = new Spindexer(hwMap, telemetry, this);
        collector = new Collector(hwMap, telemetry);
        shooter = new OneWShooter(hwMap, telemetry);
        ramp = new Ramp(hwMap, telemetry);
        pivot = new Pivot(hwMap, telemetry);
        limelight = new Limelight(hwMap, telemetry, this);

        ballTracker = new BallTrackerNew(spindexer);


        subsystems.add(limelight);

        subsystems.add(spindexer);
        subsystems.add(collector);
        subsystems.add(shooter);
//        subsystems.add(finger);
//        subsystems.add(shooterOne);
        subsystems.add(ramp);
        subsystems.add(pivot);

        // Defining the Motors
        drive = new MecanumDrive(hwMap,startPose);

    }


    public void update() {
        String newBall;
        for (Component c : subsystems) {
            c.update();
        }


        drive.localizer.update();

//
        if (shooter.shooterState == OneWShooter.ShooterState.SHOOT_CLOSE || shooter.shooterState == OneWShooter.ShooterState.SHOOT_FAR) {

            pivot.updateCompensatedPosition(shooter.shotsFired);
            telemetry.addLine("OKKKKKKKKKk");
        } else {
            shooter.resetShotCounter();
        }

        if (limelight != null) {
            limelight.update();
        }
        isSpindStopped = (Math.abs(spindexer.spindexerPid.getTarget() - spindexer.getCurrentPosition())) < 400 ;
        ballSensor.setIfIndexerIsMoving(!isSpindStopped);

        // DETECT BALL IF SPIND IS NOT MOVING
        if (isSpindStopped ) {
            newBall = ballSensor.scanForNewBall();

            if (newBall != null ) {

                BallTrackerNew.BallColor color = BallTrackerNew.BallColor.valueOf(newBall);

                BallTrackerNew.Slot collectSlot = limelight.ballTrackerNew.getSlotAtCollectPos();
                collectSlot.color = color;

                if (limelight.ballTrackerNew.isNextSlotEmpty()) {
//                    spindexer.setTargetAdj(j.spindexerConstants.TICKS_120);
                }
            }
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) ballSensor.colorSensor).getDistance(DistanceUnit.CM));
        }

        isNextEmpty = limelight.ballTrackerNew.isNextSlotEmpty();

        // ANTIJAMMM

//        if (ballSensor.isDistanceGreaterThanSeven() && spindexer.isJammed()) {
//
//        }


        // clearing ball tracking
        if (shooter.shotsFired == 3) {
            ballTracker.removeAll();
        }

        // ADD BALL TO BALL TRACK + MOVE SPINDEXER (CAN BE REMOVED)
        // TODO: fine tune settle time
//        if (goodToMove && ballDetectTimer.milliseconds() > 0) {
//
//            BallTrackerNew.Slot collectSlot = limelight.ballTrackerNew.getSlotAtCollectPos();
//            collectSlot.color = detectedColor;
//            if (isNextEmpty) {spindexer.setTargetAdj(Constants.spindexerConstants.TICKS_120);}
//            goodToMove = false;
//            detectedColor = null;
//        }






        // CHECK IF SPINDEXER JUST FINISHED MOVING!!! ========================================================

//        if (spindexer.justFinishedMoving) {
//            checkingColorAfterMovingSpind = true;
//
//            spindexer.justFinishedMoving = false;
//        }
//
//        // CHECK COLOR AFTER MOVEMENT!!! ======================================================== AhHHHh PROBLEM FIX
//        if (checkingColorAfterMovingSpind) {
//            String colorAfterMovingSpind = ballSensor.checkColorAfterMovement();
//
//            if (colorAfterMovingSpind != null) {
//                checkingColorAfterMovingSpind = false;
//
//                telemetry.addData("after movement clr", colorAfterMovingSpind);
//                if (colorAfterMovingSpind.equals("EMPTY")) {
//
//                    telemetry.addLine("empty");
//                } else {
//
//                    telemetry.addData("not empty!? ball color", colorAfterMovingSpind);
//                }
//            }
//        }

        // TELEMETRY ===============================================================================
        allTelemetry();
        telemetry.update();
    }



    private void allTelemetry() {

        telemetry.addLine("=== SPINDEXER SLOTS ===");
        telemetry.addData("Slot A", String.format("%s @ %d ticks", limelight.ballTrackerNew.slotA.color, limelight.ballTrackerNew.slotA.currentAbsPos));
        telemetry.addData("Slot B", String.format("%s @ %d ticks", limelight.ballTrackerNew.slotB.color, limelight.ballTrackerNew.slotB.currentAbsPos));
        telemetry.addData("Slot C", String.format("%s @ %d ticks", limelight.ballTrackerNew.slotC.color, limelight.ballTrackerNew.slotC.currentAbsPos));

        telemetry.addLine("\n=== WHERE IS EACH SLOT ===");
        telemetry.addData("At Collect Pos", limelight.ballTrackerNew.getSlotAtCollectPos().name);
        telemetry.addData("At Shooting Pos", limelight.ballTrackerNew.getSlotAtShootingPos().name);

        telemetry.addLine("\n=== DETECTION STATES");
        telemetry.addData("Spind Stopped?", isSpindStopped);
        telemetry.addData("Good To Move?", goodToMove);
        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("Collect color - ball tracking", limelight.ballTrackerNew.thisBall);
        telemetry.addData("Is next empty", isNextEmpty);
        telemetry.addData("Color delay time", BallSensor.delayTimeMs);
        telemetry.addData("from delay settle time", BallSensor.settleDelayMs);

        telemetry.addLine("\n === PATTERN MATCHING ===");
        telemetry.addData("Target Motif", limelight.ballTrackerNew.targetMotif);
        telemetry.addData("Fiducial ID", Limelight.feducialResult);


        telemetry.addLine("\n=== COLOR SENSOR ===");
        telemetry.addData("R", ballSensor.rPercent);
        telemetry.addData("G", ballSensor.gPercent);
        telemetry.addData("B", ballSensor.bPercent);
        telemetry.addData("Alpha", ballSensor.alpha);

        telemetry.addLine("\n=== LOCATION ===");
        telemetry.addData("Pose", drive.localizer.getPose().toString());
        telemetry.addData("x", drive.localizer.getPose().position.x);
        telemetry.addData("y", drive.localizer.getPose().position.y);
        telemetry.addData("heading", drive.localizer.getPose().heading.toDouble());

        telemetry.addLine("\n=== SHOOTER ===");
        telemetry.addData("State", shooter.shooterState);
        telemetry.addData("Shooter Vel", shooter.shooterMotorOne.getVelocity());
        telemetry.addData("Shooter Target", shooter.shooterPID.getTarget());
        telemetry.addData("At Speed", Math.abs(shooter.shooterMotorOne.getVelocity() - shooter.targetVel) < 50);
        telemetry.addData("Giving power", shooter.shooterMotorOne.getPower());
        telemetry.addData("Shots Fired", shooter.shotsFired);

        telemetry.addLine("\n=== SPINDEXER ===");
        telemetry.addData("Position", spindexer.getCurrentPosition());
        telemetry.addData("Target", spindexer.spindexerPid.getTarget());
        telemetry.addData("Power giving:", spindexer.spindexerMotor.getPower());

        telemetry.addLine("\n=== HOOD ===");
        telemetry.addData("Pivot left pos", pivot.getLeftPos());
        telemetry.addData("Pivot right pos", pivot.getRightPos());
        telemetry.addData("Pivot state", pivot.pivotState);


        telemetry.addLine("\n=== INTAKE + RAMP ===");
        telemetry.addData("Collector", collector.collectorMotor.getVelocity());
        telemetry.addData("Ramp", ramp.rampState);







    }

}