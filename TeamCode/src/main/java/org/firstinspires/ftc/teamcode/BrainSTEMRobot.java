package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Finger;
import org.firstinspires.ftc.teamcode.utils.Component;

public class BrainSTEMRobot {

    public static double autoX, autoY, autoH;
    // Don't touch these
    public Telemetry telemetry;
    public OpMode opMode;
    private final ArrayList<Component> subsystems;
    public Spindexer spindexer;
    public Collector collector;
    public Shooter shooter;

    public Finger finger;
    public MecanumDrive drive;

    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, OpMode opMode, Pose2d startPose) {

        this.telemetry = telemetry;
        this.opMode = opMode;
        subsystems = new ArrayList<>();

        spindexer = new Spindexer(hwMap, telemetry, this);
        collector = new Collector(hwMap, telemetry);
        shooter = new Shooter(hwMap, telemetry);
        finger = new Finger(hwMap, telemetry, this);

        subsystems.add(spindexer);
        subsystems.add(collector);
        subsystems.add(shooter);
        subsystems.add(finger);

        // Defining the Motors
        drive = new MecanumDrive(hwMap,startPose);
    }

    public void update() {
        for (Component c : subsystems) {
            c.update();
        }
        drive.localizer.update();
    }
}