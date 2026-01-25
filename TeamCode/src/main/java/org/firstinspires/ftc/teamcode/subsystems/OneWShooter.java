package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Component;
import org.firstinspires.ftc.teamcode.utils.PIDController;



/*

@Config
public class OneWShooter implements Component {

    private final Telemetry telemetry;

    public static boolean powerMotors = true;


    // hardware constants

    private HardwareMap map;
    public DcMotorEx shooterMotorTwo;
    public DcMotorEx shooterMotorOne;


    public ShooterState shooterState;

    //PID Controllers
    public PIDController shooterPID;


    public enum ShooterState {
        OFF,
        IDLE,
        SHOOT_FAR,
        SHOOT_CLOSE,
        AUTO
    }


    public OneWShooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;

        shooterMotorOne = hardwareMap.get(DcMotorEx.class, "shooterMotorOne");
        shooterMotorTwo = hardwareMap.get(DcMotorEx.class, "shooterMotorTwo");

        shooterMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD); // change
        shooterMotorOne.setDirection(DcMotorSimple.Direction.REVERSE); // change

        shooterMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//
//        shooterPID = new PIDController(Constants.ShooterConstants.kP_ONE, Constants.ShooterConstants.kI, Constants.ShooterConstants.kD);
//        shooterPID.setInputBounds(0, Constants.ShooterConstants.MAX_TICKS_PER_SEC);
//        shooterPID.setOutputBounds(0,1);

        PIDFCoefficients newPIDF = new PIDFCoefficients(
                Constants.ShooterConstants.kP_ONE,
                Constants.ShooterConstants.kI,
                Constants.ShooterConstants.kD,
                Constants.ShooterConstants.kF
        );
        shooterMotorOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
        shooterMotorTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);


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
                break;
            case SHOOT_FAR:
                setBoth(Constants.ShooterConstants.FAR_SHOOT_VEL);
                break;
            case SHOOT_CLOSE:
                setBoth(Constants.ShooterConstants.CLOSE_SHOOT_VEL);
                break;
            case IDLE:
                shooterMotorOne.setPower(Constants.ShooterConstants.IDLE_POWER);
                shooterMotorTwo.setPower(Constants.ShooterConstants.IDLE_POWER);

                break;
            case AUTO:
                setBoth(Constants.ShooterConstants.AUTO_VEL);
                break;

        }

        telemetry.addData("shooter motor state", shooterState);

    }
    public void setBothMotorVelocities(double targetVelocity) {
        shooterPID.setTarget(targetVelocity);
        double error = targetVelocity - Math.abs(shooterMotorOne.getVelocity());

        double pidOutput = shooterPID.updateWithError(error);

        double shooterPower = pidOutput + Constants.ShooterConstants.kV1 * targetVelocity;

        shooterPower = Range.clip(shooterPower, 0, Constants.ShooterConstants.MAX_POWER);

        if (powerMotors) {
            shooterMotorOne.setPower(shooterPower);
            shooterMotorTwo.setPower(shooterPower);
        }

        telemetry.addData("Shooter Target Vel", targetVelocity);
        telemetry.addData("shooter error 1", error);
        telemetry.addData("shooter motor one velocity", shooterMotorOne.getVelocity());
        telemetry.addData("shooter motor two velocity", shooterMotorTwo.getVelocity());

        telemetry.addData("Shooter PID output 1", pidOutput);
        telemetry.addData("Shooter total output 1", shooterPower);

        telemetry.addData("shooter one Power", shooterMotorOne.getPower());
        telemetry.addData("shooter two power", shooterMotorTwo.getPower());



    }

    public void setBoth(double power) {
        shooterMotorTwo.setVelocity(power);
        shooterMotorOne.setVelocity(power);
    }
    public void setShooterShootFar() {
        shooterState = ShooterState.SHOOT_FAR;
//        shooterPID.reset();
    }
    public void setShooterShootClose() {
        shooterState = ShooterState.SHOOT_CLOSE;
//        shooterPID.reset();
    }
    public void setShooterOff() {
        shooterState = ShooterState.OFF;
//        shooterPID.reset();
    }

    public void setShooterIdle(){
        shooterState = ShooterState.IDLE;
//        shooterPID.reset();
    }

    @Override
    public String test() {
        return null;
    }
}

 */
