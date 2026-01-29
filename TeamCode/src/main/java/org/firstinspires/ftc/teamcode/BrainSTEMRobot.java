package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.subsystems.OneWShooter;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.sensors.Limelight;
import org.firstinspires.ftc.teamcode.utils.BallSensor;
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
    public Ramp ramp;
    public OneWShooter shooter;

    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, Pose2d startPose) {

        this.telemetry = telemetry;
        this.opMode = opMode;

        ballSensor = new BallSensor(hwMap);
        subsystems = new ArrayList<>();

        spindexer = new Spindexer(hwMap, telemetry, this);
        collector = new Collector(hwMap, telemetry);
//        shooter = new Shooter(hwMap, telemetry);
        shooter = new OneWShooter(hwMap, telemetry);
        ramp = new Ramp(hwMap, telemetry);
        pivot = new Pivot(hwMap, telemetry);
//        finger = new Finger(hwMap, telemetry, this);
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

        String newBall = ballSensor.scanForNewBall();

        if (spindexer.isStatic() && newBall != null) {
            telemetry.addData("New Ball Detected", newBall);
            limelight.ballTracker.addBall(newBall);
//
//            spindexer.setSpindexerTargetAdjustment(48);
//            limelight.ballTracker.rotated60();
        }
    }
}