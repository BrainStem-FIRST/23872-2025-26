package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Component;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.Constants;

/*
@Disabled
public class Shooter implements Component {

    // setMotorEnable to compensate for voltage
    // write pidf stuff that accounts for voltage

    private final Telemetry telemetry;

    public static boolean powerMotors = true;


    // hardware constants

    private HardwareMap map;
    public double avgMotorVel;
    public DcMotorEx shooterMotorTwo;
    public DcMotorEx shooterMotorOne;


    public ShooterState shooterState;

    //PID Controllers
    public PIDController shooterPID1, shooterPID2;
//    public PIDController shooterPID2;

    public enum ShooterState {
        OFF,
        IDLE,
        SHOOT_FAR,
        SHOOT_CLOSE,
        AUTO
    }
//      public PidDrivePIDController shooterPid;


    public  Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;

        shooterMotorOne = hardwareMap.get(DcMotorEx.class, "shooterMotorOne");
        shooterMotorTwo = hardwareMap.get(DcMotorEx.class, "shooterMotorTwo");

        shooterMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        shooterMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterPID1 = new PIDController(Constants.shooterConstants.kP_ONE, Constants.shooterConstants.kI, Constants.shooterConstants.kD);
        shooterPID1.setInputBounds(0, Constants.shooterConstants.MAX_TICKS_PER_SEC);
        shooterPID1.setOutputBounds(0,1);

        shooterPID2 = new PIDController(Constants.shooterConstants.kP_TWO, Constants.shooterConstants.kI, Constants.shooterConstants.kD);
        shooterPID2.setInputBounds(0, Constants.shooterConstants.MAX_TICKS_PER_SEC);
        shooterPID2.setOutputBounds(0,1);

        this.shooterState = ShooterState.OFF;
    }



    @Override
    public void reset() {
        shooterPID1.reset();
        shooterPID2.reset();
    }


    @Override
    public void update() {
        avgMotorVel = (Math.abs(shooterMotorOne.getVelocity()) + Math.abs(shooterMotorTwo.getVelocity())) / 2.;
        switch (shooterState) {
            case OFF:
                shooterMotorOne.setPower(0);
                shooterMotorTwo.setPower(0);
                break;
            case SHOOT_FAR:
                setBothMotorVelocities(Constants.shooterConstants.FAR_SHOOT_VEL);
                break;
            case SHOOT_CLOSE:
                setBothMotorVelocities(Constants.shooterConstants.CLOSE_SHOOT_VEL);
                break;
            case IDLE:
                shooterMotorOne.setPower(Constants.shooterConstants.IDLE_POWER);
                shooterMotorTwo.setPower(Constants.shooterConstants.IDLE_POWER);

                break;
            case AUTO:
                setBothMotorVelocities(Constants.shooterConstants.AUTO_VEL);
                break;

        }

//        shooterMotorTwo.setPower(shooterMotorOne.getPower());

//        telemetry.addData("avg motor vel", avgMotorVel);

        telemetry.addData("shooter motor state", shooterState);

    }
    public void setBothMotorVelocities(double targetVelocity) {
        shooterPID1.setTarget(targetVelocity);
        shooterPID2.setTarget(targetVelocity);
        double error1 = targetVelocity - Math.abs(shooterMotorOne.getVelocity());
        double error2 = targetVelocity - Math.abs(shooterMotorTwo.getVelocity());

        double pidOutput1 = shooterPID1.updateWithError(error1);
        double pidOutput2 = shooterPID2.updateWithError(error2);

        double shooterOnePower = pidOutput1 + Constants.shooterConstants.kV1 * targetVelocity;
        double shooterTwoPower = pidOutput2 + Constants.shooterConstants.kV2 * targetVelocity;

        shooterOnePower = Range.clip(shooterOnePower, 0, Constants.shooterConstants.MAX_POWER);
        shooterTwoPower = Range.clip(shooterTwoPower, 0, Constants.shooterConstants.MAX_POWER);

        if (powerMotors) {
            shooterMotorOne.setPower(shooterOnePower);
            shooterMotorTwo.setPower(shooterTwoPower);
        }

    }

    public void setShooterShootFar() {
        shooterState = ShooterState.SHOOT_FAR;
        shooterPID1.reset();
        shooterPID2.reset();
    }
    public void setShooterShootClose() {
        shooterState = ShooterState.SHOOT_CLOSE;
        shooterPID1.reset();
        shooterPID2.reset();
    }
    public void setShooterOff() {
        shooterState = ShooterState.OFF;
        shooterPID1.reset();
        shooterPID2.reset();
    }

    public void setShooterIdle(){
        shooterState = ShooterState.IDLE;
        shooterPID1.reset();
        shooterPID2.reset();
    }

    @Override
    public String test() {
        return null;
    }
}

 */
