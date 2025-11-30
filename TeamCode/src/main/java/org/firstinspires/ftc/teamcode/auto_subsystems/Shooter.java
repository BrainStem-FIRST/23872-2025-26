package org.firstinspires.ftc.teamcode.auto_subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;

@Config
public class Shooter implements Component {
    public static double SHOOTER_TICKS_PER_REVOLUTION = 288;
    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx shooterMotorTwo;
    public DcMotorEx shooterMotorOne;

    public ShooterState shooterState;

    public double targetVelocity;

    public enum ShooterState {
        OFF,
        SHOOT_FAR,
        SHOOT_CLOSE,
        PRESPIN
    }

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;

        shooterMotorOne = map.get(DcMotorEx.class, "shooterMotorOne");
        shooterMotorTwo = map.get(DcMotorEx.class, "shooterMotorTwo");


        shooterMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.shooterState = ShooterState.OFF;
    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {
        switch (shooterState) {
            case OFF:
                shooterMotorOne.setVelocity(0);
                shooterMotorTwo.setVelocity(0);

                break;
            case SHOOT_FAR:
                shooterMotorOne.setVelocity(4000);
                shooterMotorTwo.setVelocity(4000);

                break;
            case SHOOT_CLOSE:
                shooterMotorOne.setVelocity(1);
                shooterMotorTwo.setVelocity(1);
                break;
            case PRESPIN:
                shooterMotorOne.setVelocity(1000);
                shooterMotorTwo.setVelocity(1000);
                break;
        }

        telemetry.addData("shooter motor one velocity", shooterMotorOne.getVelocity());
        telemetry.addData("shooter motor two velocity", shooterMotorTwo.getVelocity());
    }



    public void setShooterShootFar(){
        shooterState = ShooterState.SHOOT_FAR;
    }

    public void setShooterOff(){
        shooterState = ShooterState.OFF;
    }

    @Override
    public String test() {
        return null;
    }
}