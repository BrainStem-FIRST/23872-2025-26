package org.firstinspires.ftc.teamcode.tuning_opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp
public class shooterTuning extends OpMode {

    public static double kP = 0; // change
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 41.63; //Adjust this first?
    public double highVelocity = 700;
    public double lowVelocity = 500;
    double currentTarget = highVelocity;

    double[] stepSizes = {10.0, 1.0, 0.1 , 0.01, 0.001, 0.0001};

    int stepIndex = 1;
    public DcMotorEx shooterMotorTwo;
    public DcMotorEx shooterMotorOne;
    public void init() {
        shooterMotorOne = hardwareMap.get(DcMotorEx.class, "shooterMotorOne");
        shooterMotorTwo = hardwareMap.get(DcMotorEx.class, "shooterMotorTwo");

        shooterMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(
                kP,
                0,
                0,
                kF
        );

        shooterMotorOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooterMotorTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        telemetry.addLine("init complete");

    }

    public void loop(){

        if (gamepad1.yWasPressed()) {
            if (currentTarget ==highVelocity) {
                currentTarget = lowVelocity;
            } else {
                currentTarget = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            kF -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            kF += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            kP -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            kP += stepSizes[stepIndex];
        }

        //set new PIDF coeeficients
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(
                kP,
                0,
                0,
                kF
        );

        shooterMotorOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooterMotorTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        //set velocity

        shooterMotorOne.setVelocity(currentTarget);
        shooterMotorTwo.setVelocity(currentTarget);

        double curVelocity1 = shooterMotorOne.getVelocity();
        double curVelocity2 = shooterMotorTwo.getVelocity();

        double error1 = currentTarget - curVelocity1;
        double error2 = currentTarget - curVelocity2;

        //telemtry

        telemetry.addData("Target Velocity", currentTarget);
        telemetry.addData("Current Velocity 1", "%.2f", curVelocity1);
        telemetry.addData("Current Velocity 2", "%.2f", curVelocity2);
        telemetry.addData("Error 1", "%.2f", error1);
        telemetry.addData("Error 2", "%.2f", error2);
        telemetry.addLine("--------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)",kP);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)",kF);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();

    }
}
