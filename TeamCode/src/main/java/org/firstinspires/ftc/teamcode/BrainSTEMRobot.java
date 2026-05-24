package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Parking;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
//import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodLookup;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.sensors.Limelight;
import org.firstinspires.ftc.teamcode.utils.BallSensor;
import org.firstinspires.ftc.teamcode.utils.BallTracker;
import org.firstinspires.ftc.teamcode.utils.Component;

public class BrainSTEMRobot {
    public static double autoX = 0, autoY = 0, autoH = 0;
    private final ArrayList<Component> subsystems;
    public OpMode opMode; Telemetry telemetry; Spindexer spindexer;  Collector collector; MecanumDrive drive; Limelight limelight; BallSensor ballSensor; LED led; Pivot pivot; Ramp ramp; Shooter shooter; Parking park; BallTracker ballTracker;

    public boolean hit = false, isSpindStopped,  isNextEmpty,  isAuto = false;
    public int ballsShot = 0;
    private ElapsedTime lastShotTime;
    private ShooterHoodLookup shooterHoodLookup;
    public boolean red;

    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, Pose2d startPose) {
        this.telemetry = telemetry;
        this.opMode = opMode;

        park = new Parking(hwMap, telemetry);
        led = new LED(hwMap, telemetry, this);
        ballSensor = new BallSensor(hwMap);
        subsystems = new ArrayList<>();
        spindexer = new Spindexer(hwMap, telemetry, this);
        collector = new Collector(hwMap, telemetry);
        shooter = new Shooter(hwMap, telemetry, this);
        ramp = new Ramp(hwMap, telemetry);
        pivot = new Pivot(hwMap, telemetry, shooter);
        limelight = new Limelight(hwMap, telemetry, this);
        ballTracker = new BallTracker(spindexer);
        drive = new MecanumDrive(hwMap,startPose);
        lastShotTime = new ElapsedTime();
        shooterHoodLookup = new ShooterHoodLookup();

        subsystems.add(limelight);
        subsystems.add(park);
        subsystems.add(led);
        subsystems.add(spindexer);
        subsystems.add(collector);
        subsystems.add(shooter);
        subsystems.add(ramp);
        subsystems.add(pivot);

    }

    public void update() {
        drive.localizer.update();

        Vector2d goalPosition = red ?
                new Vector2d(Constants.shooterConstants.redGoalX, Constants.shooterConstants.redGoalY) :
                new Vector2d(Constants.shooterConstants.blueGoalX, Constants.shooterConstants.blueGoalY);
        Pose2d robotPose = drive.localizer.getPose();
        Vector2d robotToGoal = goalPosition.minus(robotPose.position);
        double distFromGoal = Math.hypot(robotToGoal.x, robotToGoal.y);
        telemetry.addData("DIST FROM GOAL", distFromGoal);
        telemetry.addData("Goal pose", goalPosition.x + " " + goalPosition.y);
        shooter.closeTargetSpeed = shooterHoodLookup.getShooterSpeed(distFromGoal);
        pivot.closeTargetPosition = shooterHoodLookup.getHoodPosition(distFromGoal);

        pivot.updateCompensatedPosition(ballsShot);

        for (Component c : subsystems) {
            c.update();
        }

        isSpindStopped = (Math.abs(spindexer.targetEncoder - spindexer.getCurrentPosition())) < 50 || spindexer.spindexerMotor.getVelocity()<15;
        ballSensor.setIfIndexerIsMoving(!isSpindStopped);

        String newBall = "EMPTY";
        newBall = ballSensor.scanForNewBall();

        if (isSpindStopped ) {

            if (newBall != null ) {

                BallTracker.BallColor color = BallTracker.BallColor.valueOf(newBall);

                BallTracker.Slot collectSlot = limelight.ballTrackerNew.getSlotAtCollectPos();
                collectSlot.color = color;

                telemetry.addData("Ball Color", color);
                spindexer.setTargetAdj(341);
            }
        }

        isNextEmpty = limelight.ballTrackerNew.isNextSlotEmpty();

        if (shooter.isShootFar() && ramp.isRampUp()) { ballsShot = getBallsShot();}
        else { ballsShot = 0;}

        if (shooter.isShootFar()) { Spindexer.maxPower = 0.5;}
        else if (shooter.isShootClose()){ Spindexer.maxPower = 0.99;}
        else { Spindexer.maxPower = 0.99;}

        telemetry.update();
    }

    private int getBallsShot() {
        double diff = (spindexer.wrappedEncoder - spindexer.startShootingEncoder) / 1024. * 360;
        if ( diff >= 280) {
            return 3;
        } else if ( diff >= 160) {
            return 2;
        } else if (diff >= 40) {
            return 1;
        }
        return 0;
    }

}