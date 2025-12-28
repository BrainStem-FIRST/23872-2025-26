package org.firstinspires.ftc.teamcode.tele_subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        public static double CLOSE_SHOOT_VEL = 1400;

        public static double velocityThreshold = 50; //change
        public static double shotVelDropThreshold = 70; //indicates a ball left
    }

    // hardware constants

    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx shooterMotorTwo;
    public DcMotorEx shooterMotorOne;


    public ShooterState currentState;

    //PID Controllers
    public PIDController shooterPID1;
    public PIDController shooterPID2;

    private double targetVelocity = ShooterParams.FAR_SHOOT_VEL;
    private int ballsShot = 0;

    public enum ShooterState {
        OFF,
        SHOOT_FAR,
        SHOOT_CLOSE
    }
//      public PIDController shooterPid;


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
        shooterPID2 = new PIDController(ShooterParams.kP, ShooterParams.kI, ShooterParams.kD);

        shooterMotorOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID1);
        shooterMotorTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID2);

        this.currentState = ShooterState.OFF;
    }

    public void setBothMotors(double power1, double power2) {
        shooterMotorTwo.setPower(power2);
        shooterMotorOne.setPower(power1);
    }

    @Override
    public void reset() {
        shooterPID1.reset();
        shooterPID2.reset();
        ballsShot = 0;
    }

    @Override
    public void update() {
        switch (currentState) {
            case OFF:
                setShooterVelocity(0);
                setBothMotors(0, 0);
                break;
            case SHOOT_FAR:
                setShooterVelocity(ShooterParams.FAR_SHOOT_VEL);
                doPID();

                break;
            case SHOOT_CLOSE:
                setShooterVelocity(ShooterParams.CLOSE_SHOOT_VEL);
                doPID();

                break;
        }

        telemetry.addData("shooter motor one velocity", shooterMotorOne.getVelocity());
        telemetry.addData("shooter motor two velocity", shooterMotorTwo.getVelocity());
    }

    public void setShooterVelocity(double targetVelocity) {
        shooterPID1.setTarget(targetVelocity);
        shooterPID2.setTarget(targetVelocity);
    }

    public void doPID() {
        double velocity1 = shooterMotorOne.getVelocity();
        double pid1 = shooterPID1.update(velocity1);
        double minPower1 = ShooterParams.kF * shooterPID1.getTarget(); // kF MUST be small
        double power1 = minPower1 + pid1;
        power1 = Math.max(-1.0, Math.min(1.0, power1));

        double velocity2 = shooterMotorTwo.getVelocity();
        double pid2 = shooterPID2.update(velocity2);
        double minPower2 = ShooterParams.kF * shooterPID2.getTarget(); // kF MUST be small
        double power2 = minPower2 + pid2;
        power2 = Math.max(-1.0, Math.min(1.0, power2));
        setBothMotors(power1, power2);

        telemetry.addData("Velocity", velocity1);
        telemetry.addData("FF", minPower1);
        telemetry.addData("PID", pid1);
        telemetry.addData("Power", power1);
        telemetry.addData("shooter state", currentState);
        telemetry.addData("pid targets", shooterPID1.getTarget() + " " + shooterPID2.getTarget());
    }

    public void setShooterShootFar() {
        currentState = ShooterState.SHOOT_FAR;
    }
    public void setShooterShootClose() {
        currentState = ShooterState.SHOOT_CLOSE;
    }
    public void setShooterOff() {
        currentState = ShooterState.OFF;
    }

    @Override
    public String test() {
        return null;
    }
}
