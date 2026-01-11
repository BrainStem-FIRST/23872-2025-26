package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Component;
import org.firstinspires.ftc.teamcode.utils.PIDController;


@Config
public class Shooter implements Component {

    private final Telemetry telemetry;

    public static boolean powerMotors = true;
    public static class ShooterParams {
        public  double kPOne = 0.001; // was 0.01
        public double kPTwo = 0.001;
        public  double kI = 0.0;
        public  double kD = 0.000;
        public double kV1 = 0.00049;
        public double kV2 = 0.00044;

        public double AUTO = 1150;



        public double FAR_SHOOT_VEL = 1060;
        public  double CLOSE_SHOOT_VEL = 300;
//close shoot vel og 980 DT
        public  double STOP_SHOOT = 0;

        public  double MAX_TICKS_PER_SEC = 2300;

        public  double IDLE_POWER = 0.2;
        public double maxPower = 0.99;

    }

    public static ShooterParams PARAMS = new ShooterParams();

    // hardware constants

    private HardwareMap map;
    public double avgMotorVel;
    public DcMotorEx shooterMotorTwo;
    public DcMotorEx shooterMotorOne;


    public ShooterState shooterState;

    //PID Controllers
    public PIDController shooterPID1, shooterPID2;
//    public PIDController shooterPID2;

    private int ballsShot = 0;

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

        shooterMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterPID1 = new PIDController(PARAMS.kPOne, PARAMS.kI, PARAMS.kD);
        shooterPID1.setInputBounds(0,PARAMS.MAX_TICKS_PER_SEC);
        shooterPID1.setOutputBounds(0,1);

        shooterPID2 = new PIDController(PARAMS.kPTwo, PARAMS.kI, PARAMS.kD);
        shooterPID2.setInputBounds(0,PARAMS.MAX_TICKS_PER_SEC);
        shooterPID2.setOutputBounds(0,1);

        this.shooterState = ShooterState.OFF;
    }



    @Override
    public void reset() {
        shooterPID1.reset();
        shooterPID2.reset();
        ballsShot = 0;
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
                setBothMotorVelocities(PARAMS.FAR_SHOOT_VEL);
                break;
            case SHOOT_CLOSE:
                setBothMotorVelocities(PARAMS.CLOSE_SHOOT_VEL);
                break;
            case IDLE:
                shooterMotorOne.setPower(PARAMS.IDLE_POWER);
                shooterMotorTwo.setPower(PARAMS.IDLE_POWER);

                break;
            case AUTO:
                shooterMotorOne.setPower(PARAMS.AUTO);
                shooterMotorTwo.setPower(PARAMS.AUTO);
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

        double shooterOnePower = pidOutput1 + PARAMS.kV1 * targetVelocity;
        double shooterTwoPower = pidOutput2 + PARAMS.kV2 * targetVelocity;

        shooterOnePower = Range.clip(shooterOnePower, 0, PARAMS.maxPower);
        shooterTwoPower = Range.clip(shooterTwoPower, 0, PARAMS.maxPower);

        if (powerMotors) {
            shooterMotorOne.setPower(shooterOnePower);
            shooterMotorTwo.setPower(shooterTwoPower);
        }

        telemetry.addData("Shooter Target Vel", targetVelocity);
        telemetry.addData("shooter error 1", error1);
        telemetry.addData("shooter error 2", error2);
        telemetry.addData("shooter motor one velocity", shooterMotorOne.getVelocity());
        telemetry.addData("shooter motor two velocity", shooterMotorTwo.getVelocity());

        telemetry.addData("Shooter PID output 1", pidOutput1);
        telemetry.addData("Shooter PID output 2", pidOutput2);
        telemetry.addData("Shooter total output 1", shooterOnePower);
        telemetry.addData("Shooter total output 2", shooterTwoPower);

        telemetry.addData("shooter one Power", shooterMotorOne.getPower());
        telemetry.addData("shooter two power", shooterMotorTwo.getPower());


//        shooterMotorTwo.setPower(power2);
//        shooterMotorOne.setPower(power1);

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
