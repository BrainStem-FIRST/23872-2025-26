package tele_subsystems;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

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

    public double FAR_SHOOT_VEL = 3750;
    public double CLOSE_SHOOT_VEL = 2000;

    public double CLOSE_SHOOT_VEL = 2000;
    public double FAR_SHOOT_VEL = 3750;


    public ShooterState shooterState;

    public double targetVelocity;

    public enum ShooterState {
        OFF,
        PRESPIN,
        SHOOT_FAR,
        SHOOT_CLOSE

    }

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;

        shooterMotorOne = map.get(DcMotorEx.class, "shooterMotorOne");
        shooterMotorTwo = map.get(DcMotorEx.class, "shooterMotorTwo");

        shooterMotorOne.setDirection(REVERSE);
        shooterMotorTwo.setDirection(REVERSE);

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
                shooterMotorOne.setVelocity(FAR_SHOOT_VEL);
                shooterMotorTwo.setVelocity(FAR_SHOOT_VEL);

                break;
            case SHOOT_CLOSE:
                shooterMotorOne.setVelocity(CLOSE_SHOOT_VEL);
                shooterMotorTwo.setVelocity(CLOSE_SHOOT_VEL);
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