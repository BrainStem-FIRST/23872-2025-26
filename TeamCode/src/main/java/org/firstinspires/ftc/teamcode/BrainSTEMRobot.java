package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.subsystems.OneWShooter;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
//import org.firstinspires.ftc.teamcode.subsystems.Ramp;
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
//    public Shooter shooter;
//    public Finger finger;
    public MecanumDrive drive;

    public Limelight limelight;
    public BallSensor ballSensor;
    public Pivot pivot;
//    public Ramp ramp;
    public OneWShooter shooter;

    private boolean goodToMove = false;
    private ElapsedTime moveDelayTimer = new ElapsedTime();

    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, Pose2d startPose) {

        this.telemetry = telemetry;
        this.opMode = opMode;

        ballSensor = new BallSensor(hwMap);
        subsystems = new ArrayList<>();

        spindexer = new Spindexer(hwMap, telemetry, this);
        collector = new Collector(hwMap, telemetry);
//        shooter = new Shooter(hwMap, telemetry);
        shooter = new OneWShooter(hwMap, telemetry);
//        ramp = new Ramp(hwMap, telemetry);
        pivot = new Pivot(hwMap, telemetry);
//        finger = new Finger(hwMap, telemetry, this);
        limelight = new Limelight(hwMap, telemetry, this);


        subsystems.add(limelight);

        subsystems.add(spindexer);
        subsystems.add(collector);
        subsystems.add(shooter);
//        subsystems.add(finger);
//        subsystems.add(shooterOne);
//        subsystems.add(ramp);
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

        boolean isMotorBusy = spindexer.spindexerMotor.isBusy();

        ballSensor.setIfIndexerIsMoving(isMotorBusy);



        if (!goodToMove) {
            String newBall = ballSensor.scanForNewBall();

            if (newBall != null) {
                limelight.ballTrackerNew.addBall(BallTrackerNew.BallColor.valueOf(newBall));
                goodToMove = true;
                moveDelayTimer.reset();
            }
        }



        if (goodToMove && moveDelayTimer.milliseconds() > 500) {
            // TODO: fine tune time
            spindexer.setSpindexerTargetAdjustment(Constants.SpindexerConstants.TICKS_120);
            goodToMove = false;
        }


        if (spindexer.spindexerMotor.isBusy() && Math.abs(spindexer.spindexerMotor.getCurrentPosition() - spindexer.spindexerMotor.getTargetPosition()) < 20) {

        }
    }

    private void printBallStatus() {
        BallTrackerNew ballTracker = limelight.ballTrackerNew;

        telemetry.addLine("=== SPINDEXER SLoTs ++++++");
        telemetry.addData("Slot A", ballTracker.slotA.color);
        telemetry.addData("Slot B", ballTracker.slotB.color);
        telemetry.addData("Slot C", ballTracker.slotC.color);


        telemetry.addLine("=== PATTERN MATCHING ===");
        telemetry.addData("Target Motif", ballTracker.targetMotif);
        telemetry.addData("Feducial ID", Limelight.feducialResult);

        telemetry.addLine("--- SENSOR STATE ---");
        telemetry.addData("Is indexing?", spindexer.spindexerMotor.isBusy());
        telemetry.addData("good to move?", goodToMove);

        telemetry.update();
    }

}