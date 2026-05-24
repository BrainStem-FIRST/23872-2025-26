package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Component;
import org.firstinspires.ftc.teamcode.utils.PIDController;





@Config
public class Shooter implements Component {
    private Telemetry telemetry; DcMotorEx shooterMotorTwo, shooterMotorOne; HardwareMap map; ShooterState shooterState; PIDController shooterPID; BrainSTEMRobot robot;
    private static boolean powerMotors = true;
    public double targetVel, currentVel1,  currentVel2, closeTargetSpeed, error1, error2;
    private boolean wasAtSpeed = false;
    private enum ShooterState {
        OFF,
        IDLE,
        POINT,
        SHOOT_FAR,
        SHOOT_CLOSE,
        AUTO
    }

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        this.robot = robot;
        this.map = hardwareMap;
        this.telemetry = telemetry;

        shooterMotorOne = hardwareMap.get(DcMotorEx.class, "shooterMotorOne");
        shooterMotorTwo = hardwareMap.get(DcMotorEx.class, "shooterMotorTwo");

        shooterMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterPID = new PIDController(Constants.shooterConstants.kP_ONE, Constants.shooterConstants.kI, Constants.shooterConstants.kD);
        shooterPID.setInputBounds(0, Constants.shooterConstants.MAX_TICKS_PER_SEC);
        shooterPID.setOutputBounds(0,1);

        this.shooterState = ShooterState.OFF;
    }

    @Override
    public void reset() {
       shooterPID.reset();
    }

    @Override
    public void update() {
        switch (shooterState) {
            case OFF:
                shooterMotorOne.setPower(0);
                shooterMotorTwo.setPower(0);

                targetVel = 0;
                break;
            case SHOOT_FAR:
                shooterPID.setPIDValues(Constants.shooterConstants.kP_TWO, Constants.shooterConstants.kI, Constants.shooterConstants.kD);
                setBothMotorVelocities(Constants.shooterConstants.FAR_SHOOT_VEL);
                targetVel = Constants.shooterConstants.FAR_SHOOT_VEL;
                break;

            case POINT:
                shooterPID.setPIDValues(Constants.shooterConstants.kP_TWO, Constants.shooterConstants.kI, Constants.shooterConstants.kD);
                setBothMotorVelocities(Constants.shooterConstants.POINT_SHOOT_VEL);
                targetVel = Constants.shooterConstants.POINT_SHOOT_VEL;
                break;
            case SHOOT_CLOSE:
                shooterPID.setPIDValues(Constants.shooterConstants.kP_ONE, Constants.shooterConstants.kI, Constants.shooterConstants.kD);
                targetVel = closeTargetSpeed;
                setBothMotorVelocities(targetVel);
                break;
            case IDLE:
                shooterMotorOne.setPower(Constants.shooterConstants.IDLE_POWER);
                shooterMotorTwo.setPower(Constants.shooterConstants.IDLE_POWER);
                targetVel = 0;
                break;
            case AUTO:
                setBothMotorVelocities(Constants.shooterConstants.CLOSE_SHOOT_VEL);
                targetVel = Constants.shooterConstants.AUTO_VEL;
                break;
        }

        currentVel1 = Math.abs(shooterMotorOne.getVelocity());
        currentVel2 = Math.abs(shooterMotorTwo.getVelocity());

        error1 = Math.abs(currentVel1 - targetVel);
        error2 = Math.abs(currentVel2 - targetVel);

        double currentVel = shooterMotorOne.getVelocity();
        boolean isAtSpeed = Math.abs(currentVel - targetVel) < 50;


    }

    public boolean isUpToSpeed() {
        return (Math.abs(shooterMotorOne.getVelocity() - shooterPID.getTarget()) < 50) && shooterPID.getTarget()!= 0;
    }


    public void setBothMotorVelocities(double targetVelocity) {
        shooterPID.setTarget(targetVelocity);
        double error = targetVelocity - Math.abs(shooterMotorOne.getVelocity());

        double pidOutput = shooterPID.updateWithError(error);

        double shooterPower = pidOutput + Constants.shooterConstants.kV1 * targetVelocity;

        shooterPower = Range.clip(shooterPower, 0, Constants.shooterConstants.MAX_POWER);

        if (powerMotors) {
            shooterMotorOne.setPower(shooterPower);
            shooterMotorTwo.setPower(shooterPower);
        }


    }

    public void setBoth(double power) {
        shooterMotorTwo.setVelocity(power);
        shooterMotorOne.setVelocity(power);
    }
    public void setShooterShootFar() {
        shooterState = ShooterState.SHOOT_FAR;
        shooterPID.reset();
    }
    public void setShooterShootAuto() {
        shooterState = ShooterState.AUTO;
        shooterPID.reset();
    }
    public void setShooterShootClose() {
        shooterState = ShooterState.SHOOT_CLOSE;
        shooterPID.reset();
    }
    public void setShooterShootPoint() {
        shooterState = ShooterState.POINT;
        shooterPID.reset();
    }

    public boolean   isShootFar() {
        if (shooterState == ShooterState.SHOOT_FAR) {
            return true;
        }
        return false;
    }

    public boolean   isShootClose() {
        if (shooterState == ShooterState.SHOOT_CLOSE) {
            return true;
        }
        return false;
    }
    public void setShooterOff() {
        shooterState = ShooterState.OFF;
        shooterPID.reset();
    }

    public void setShooterIdle(){
        shooterState = ShooterState.IDLE;
        shooterPID.reset();
    }




    @Override
    public String test() {
        return null;
    }
}


