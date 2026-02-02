package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
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

    private boolean goodToMove = false;
    private BallTrackerNew.BallColor detectedColor;
    private ElapsedTime ballDetectTimer = new ElapsedTime();

    public boolean isSpindStopped;


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
        for (Component c : subsystems) {
            c.update();
        }
        drive.localizer.update();

        if (limelight != null) {
            limelight.update();
        }

        // GIVE SPIND STATE TO BALL SENSOR
        isSpindStopped = (Math.abs(spindexer.spindexerPid.getTarget() - spindexer.spindexerMotor.getCurrentPosition())) < 150; // change val
        ballSensor.setIfIndexerIsMoving(!isSpindStopped);

        // DETECT BALL IF SPIND IS NOT MOVING
        if (isSpindStopped && !goodToMove) {
            String newBall = ballSensor.scanForNewBall();
            if (newBall != null && !newBall.equals("EMPTY")) {
                detectedColor = BallTrackerNew.BallColor.valueOf(newBall);
                goodToMove = true;
                ballDetectTimer.reset();
            }
        }

        // ADD BALL TO BALL TRACK + MOVE SPINDEXER (CAN BE REMOVED)
        // TODO: fine tune settle time
        if (goodToMove && ballDetectTimer.milliseconds() > 50) {
            limelight.ballTrackerNew.addBall(detectedColor);
//            spindexer.setTargetAdj(Constants.spindexerConstants.TICKS_120);
            goodToMove = false;
            detectedColor = null;
        }




        // CHECK IF SPINDEXER JUST FINISHED MOVING!!! ========================================================

        if (spindexer.justFinishedMoving) {
            checkingColorAfterMovingSpind = true;

            spindexer.justFinishedMoving = false;
        }

        // CHECK COLOR AFTER MOVEMENT!!! ======================================================== AhHHHh
        if (checkingColorAfterMovingSpind) {
            String colorAfterMovingSpind = ballSensor.checkColorAfterMovement();

            if (colorAfterMovingSpind != null) {
                checkingColorAfterMovingSpind = false;

                telemetry.addData("after movement clr", colorAfterMovingSpind);
                if (colorAfterMovingSpind.equals("EMPTY")) {

                    telemetry.addLine("empty");
                } else {

                    telemetry.addData("not empty!? ball color", colorAfterMovingSpind);
                }
            }
        }

        // TELEMETRY ===============================================================================
        allTelemetry();
        telemetry.update();
    }

    private void allTelemetry() {
        BallTrackerNew ballTracker = limelight.ballTrackerNew;

        telemetry.addLine("=== SPINDEXER SLoTs ++++++");
        telemetry.addData("Slot A", ballTracker.slotA.color);
        telemetry.addData("Slot B", ballTracker.slotB.color);
        telemetry.addData("Slot C", ballTracker.slotC.color);


        telemetry.addLine("=== PATTERN MATCHING ===");
        telemetry.addData("Target Motif", ballTracker.targetMotif);
        telemetry.addData("Feducial ID", Limelight.feducialResult);

        telemetry.addLine("--- SENSOR STATE ---");
        telemetry.addData("Is indexing?", !isSpindStopped);
        telemetry.addData("good to move?", goodToMove);

        telemetry.addLine("\n=== DRIVE ===");
        telemetry.addData("Pose", drive.localizer.getPose().toString());

        telemetry.addLine("\n=== SHOOTER ===");
        telemetry.addData("State", shooter.shooterState);
        telemetry.addData("Vel", shooter.shooterMotorOne.getVelocity());
        telemetry.addData("At Speed", Math.abs(shooter.shooterMotorOne.getVelocity() - shooter.targetVel) < 50);
        telemetry.addData("Giving power", shooter.shooterMotorOne.getPower());

        telemetry.addLine("\n=== SPINDEXER ===");
        telemetry.addData("State", spindexer.spindexerState);
        telemetry.addData("Position", spindexer.getCurrentPosition());
        telemetry.addData("Target", spindexer.spindexerPid.getTarget());
        telemetry.addData("Current (mA)", spindexer.spindexerMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("AntiJam Timer", spindexer.antijamTimer);
        telemetry.addData("Power giving - PID", spindexer.updateIndexerPosition());
        telemetry.addData("Power giving - MOTOR", spindexer.spindexerMotor.getPower());

        telemetry.addData("PIVOT left pos", pivot.getLeftPos());
        telemetry.addData("pIVOT right pos", pivot.getRightPos());


        telemetry.addLine("\n=== SUBSYSTEMS ===");
        telemetry.addData("Collector", collector.collectorState);
//            telemetry.addData("Ramp", robot.ramp.rampState);

        telemetry.addData("R", ballSensor.rPercent);
        telemetry.addData("G", ballSensor.gPercent);
        telemetry.addData("B", ballSensor.bPercent);

        telemetry.addData("Detected Color", ballSensor.detectColor());





    }

}