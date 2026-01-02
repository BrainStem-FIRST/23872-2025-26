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



    public static class ShooterParams {
        public static double kP = 0.01; // change
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.00035; //Adjust this first?

        public static double FAR_SHOOT_VEL = 2000;
        public static double CLOSE_SHOOT_VEL = 1500;

        public static double STOP_SHOOT = 0;

        public static double MAX_TICKS_PER_SEC = 2300;

        public static double IDLE_SHOOT= 400;

        public static double velocityThreshold = 50; //change
        public static double shotVelDropThreshold = 70; //indicates a ball left

        public static double IDLE_POWER = 0.25;

    }

    public static ShooterParams PARAMS = new ShooterParams();

    // hardware constants

    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx shooterMotorTwo;
    public DcMotorEx shooterMotorOne;


    public ShooterState shooterState;

    //PID Controllers
    public PIDController shooterPID1;
    public PIDController shooterPID2;

    private double targetVelocity = ShooterParams.FAR_SHOOT_VEL;
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

        shooterMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterPID1 = new PIDController(ShooterParams.kP, ShooterParams.kI, ShooterParams.kD);
        shooterPID1.setInputBounds(0,ShooterParams.MAX_TICKS_PER_SEC);
        shooterPID1.setOutputBounds(0,1);
        shooterPID2 = new PIDController(ShooterParams.kP, ShooterParams.kI, ShooterParams.kD);

        shooterMotorOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID1);
        shooterMotorTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID2);

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
        switch (shooterState) {
            case OFF:
                setBothMotors(ShooterParams.STOP_SHOOT);
                break;
            case SHOOT_FAR:
                setBothMotors(ShooterParams.FAR_SHOOT_VEL);
                break;
            case SHOOT_CLOSE:
                setBothMotors(ShooterParams.CLOSE_SHOOT_VEL);
                break;
            case IDLE:
                shooterMotorOne.setPower(ShooterParams.IDLE_POWER);
                shooterMotorTwo.setPower(ShooterParams.IDLE_POWER);

                break;

        }

        shooterMotorTwo.setPower(shooterMotorOne.getPower());

        telemetry.addData("shooter motor one velocity", shooterMotorOne.getVelocity());
        telemetry.addData("shooter motor two velocity", shooterMotorTwo.getVelocity());
        telemetry.addData("shooter one Power", shooterMotorOne.getPower());
        telemetry.addData("shooter two power", shooterMotorTwo.getPower());
        telemetry.addData("shooter motor state", shooterState);

    }

    public void setShooterVelocity(double targetVelocity) {
        shooterPID1.setTarget(targetVelocity);
        shooterPID2.setTarget(targetVelocity);
    }
    public void setBothMotors(double targetVelocity) {
        shooterPID1.setTarget(targetVelocity);
        double power = shooterPID1.update(shooterMotorOne.getVelocity());
        telemetry.addData("Shooter Calculated PID Power", power);
//        power = Math.max(0, Math.min(1.0, power));
        telemetry.addData("Shooter Adjusted PID Power", power);
        shooterMotorOne.setPower(-power);
        shooterMotorTwo.setPower(-power);

//        shooterMotorTwo.setPower(power2);
//        shooterMotorOne.setPower(power1);

    }
    public void doPID() {
        double velocity1 = shooterMotorOne.getVelocity();
        double pid1 = shooterPID1.update(velocity1);
        double minPower1 = ShooterParams.kF * shooterPID1.getTarget(); // kF MUST be small
        double power1 = minPower1 + pid1;
        power1 = Math.max(-1.0, Math.min(1.0, power1));


        shooterMotorOne.setPower(power1);
        shooterMotorTwo.setPower(power1);


        telemetry.addData("Shooter Velocity", velocity1);
        telemetry.addData("Shooter FF", minPower1);
        telemetry.addData("Shooter PID", pid1);
        telemetry.addData("shooter one Power", shooterMotorOne.getPower());
        telemetry.addData("shooter two power", shooterMotorTwo.getPower());
        telemetry.addData("shooter state", shooterState);
        telemetry.addData("shooter pid targets", shooterPID1.getTarget() + " " + shooterPID2.getTarget());
    }

    public void setShooterShootFar() {
        shooterState = ShooterState.SHOOT_FAR;
    }
    public void setShooterShootClose() {
        shooterState = ShooterState.SHOOT_CLOSE;
    }
    public void setShooterOff() {
        shooterState = ShooterState.OFF;
    }

    public void setShooterIdle(){
        shooterState = ShooterState.IDLE;
    }

    @Override
    public String test() {
        return null;
    }
}
