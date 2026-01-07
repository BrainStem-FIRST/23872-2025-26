package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rr.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.util.Component;
import org.firstinspires.ftc.teamcode.util.PIDController;


@Config
public class Shooter implements Component {

    private final Telemetry telemetry;

    public static class ShooterParams {
        public  double kP = 0.0001; // was 0.01
        public  double kI = 0.0;
        public  double kD = 0.001;
        public double kV = 0.00035;
        //was .00035
        public int avgMotorDir = -1;
        public int motorPowerDir = 1;



        public  double FAR_SHOOT_VEL = 1300;
        public  double CLOSE_SHOOT_VEL = 1375;
//close shoot vel og 1500
        public  double STOP_SHOOT = 0;

        public  double MAX_TICKS_PER_SEC = 2300;

        public  double IDLE_POWER = 0.2;

    }

    public static ShooterParams PARAMS = new ShooterParams();

    // hardware constants

    private HardwareMap map;
    public double avgMotorVel;
    public DcMotorEx shooterMotorTwo;
    public DcMotorEx shooterMotorOne;


    public ShooterState shooterState;

    //PID Controllers
    public PIDController shooterPID;
//    public PIDController shooterPID2;

    private double targetVelocity = PARAMS.FAR_SHOOT_VEL;
    private int ballsShot = 0;

    public enum ShooterState {
        OFF,
        IDLE,
        SHOOT_FAR,
        SHOOT_CLOSE
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

        shooterPID = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);
        shooterPID.setInputBounds(0,PARAMS.MAX_TICKS_PER_SEC);
        shooterPID.setOutputBounds(0,1);

        this.shooterState = ShooterState.OFF;
    }



    @Override
    public void reset() {
        shooterPID.reset();
//        shooterPID2.reset();
        ballsShot = 0;
    }


    @Override
    public void update() {
        avgMotorVel = PARAMS.avgMotorDir * (shooterMotorOne.getVelocity()+shooterMotorTwo.getVelocity())/2;
        switch (shooterState) {
            case OFF:
                setBothMotors(PARAMS.STOP_SHOOT);
                break;
            case SHOOT_FAR:
                setBothMotors(PARAMS.FAR_SHOOT_VEL);
                break;
            case SHOOT_CLOSE:
                setBothMotors(PARAMS.CLOSE_SHOOT_VEL);
                break;
            case IDLE:
                shooterMotorOne.setPower(PARAMS.IDLE_POWER);
                shooterMotorTwo.setPower(PARAMS.IDLE_POWER);

                break;

        }

//        shooterMotorTwo.setPower(shooterMotorOne.getPower());

        telemetry.addData("shooter motor one velocity", shooterMotorOne.getVelocity());
        telemetry.addData("shooter motor two velocity", shooterMotorTwo.getVelocity());
        telemetry.addData("avg motor vel", avgMotorVel);
        telemetry.addData("shooter one Power", shooterMotorOne.getPower());
        telemetry.addData("shooter two power", shooterMotorTwo.getPower());
        telemetry.addData("shooter motor state", shooterState);

    }
    public void setBothMotors(double targetVelocity) {
        shooterPID.setTarget(targetVelocity);
        double shooterOnePower = shooterPID.update(PARAMS.avgMotorDir * shooterMotorOne.getVelocity()) - PARAMS.kV * targetVelocity;
        double shooterTwoPower = shooterPID.update(PARAMS.avgMotorDir * shooterMotorTwo.getVelocity()) - 0.000387 * targetVelocity;

        telemetry.addData("Shooter Calculated M1 PID Power", shooterOnePower);
        telemetry.addData("Shooter Calculated M2 PID Power", shooterTwoPower);

        shooterMotorOne.setPower(-PARAMS.motorPowerDir * shooterOnePower);
        shooterMotorTwo.setPower(-PARAMS.motorPowerDir * shooterTwoPower);


//        shooterMotorTwo.setPower(power2);
//        shooterMotorOne.setPower(power1);

    }

    public void setShooterShootFar() {
        shooterState = ShooterState.SHOOT_FAR;
        shooterPID.reset();
    }
    public void setShooterShootClose() {
        shooterState = ShooterState.SHOOT_CLOSE;
        shooterPID.reset();
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
