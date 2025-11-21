package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import subsystems.Collector;
import subsystems.Shooter;
import subsystems.Spindexer;
import subsystems.Finger;

public class BrainSTEMRobot {

    // Example Motors and Servos

    public Servo exampleServo;
    public DcMotorEx exampleMotor;


    // Don't touch these
    public Telemetry telemetry;
    public OpMode opMode;
    private ArrayList<Component> subsystems;
    public Spindexer spindexer;
    public Collector collector;
    public Shooter shooter;

    public Finger finger;
    public MecanumDrive drive;





    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, OpMode opMode) {

        this.telemetry = telemetry;
        this.opMode = opMode;
        subsystems = new ArrayList<>();

        spindexer = new Spindexer(hwMap, telemetry );
        collector = new Collector(hwMap, telemetry);
        shooter = new Shooter(hwMap, telemetry);
        finger = new Finger(hwMap, telemetry);

        subsystems.add(spindexer);
        subsystems.add(collector);
        subsystems.add(shooter);
        subsystems.add(finger);

        // Defining the Motors
        drive = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));
    }

    public void update() {
        for (Component c : subsystems) {
            c.update();
        }
    }

    public void setDTMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        drive.setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void stop() {
        drive.setMotorPowers(0, 0, 0, 0);
    }
}