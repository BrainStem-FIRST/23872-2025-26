package org.firstinspires.ftc.teamcode.helping_opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

@Disabled
@TeleOp
public class ShooterTuning extends OpMode {


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
    }

    public void loop(){
        shooterMotorOne.setVelocityPIDFCoefficients(
                Constants.ShooterConstants.kP_ONE,
                Constants.ShooterConstants.kI,
                Constants.ShooterConstants.kD,
                Constants.ShooterConstants.kF
        );
        shooterMotorTwo.setVelocityPIDFCoefficients(
                Constants.ShooterConstants.kP_ONE,
                Constants.ShooterConstants.kI,
                Constants.ShooterConstants.kD,
                Constants.ShooterConstants.kF
        );
        ;
        double targetVel = 2000;

        shooterMotorOne.setVelocity(targetVel);
        shooterMotorTwo.setVelocity(targetVel);

        TelemetryPacket packet = new TelemetryPacket();


        packet.put("target velocity", targetVel);
        packet.put("actual velocity 1", shooterMotorOne.getVelocity());
        packet.put("actual velocity 2", shooterMotorTwo.getVelocity());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }
}
