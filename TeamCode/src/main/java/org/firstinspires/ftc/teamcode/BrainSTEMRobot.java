package org.firstinspires.ftc.teamcode;

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

    // Initializing the drivetrain
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;

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
        frontLeft =  (DcMotorEx)hwMap.dcMotor.get("FL");
        frontRight = (DcMotorEx)hwMap.dcMotor.get("FR");
        backLeft =   (DcMotorEx)hwMap.dcMotor.get("BL");
        backRight =  (DcMotorEx)hwMap.dcMotor.get("BR");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);



        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void update() {
        for (Component c : subsystems) {
            c.update();
        }
    }

    public void setDTMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }



}