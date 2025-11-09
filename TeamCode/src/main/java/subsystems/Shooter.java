package subsystems;

//import static subsystems.Shooter..shooterState.OFF;
//import static subsystems.Shooter..shooterState.ON;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;


@Config
public class Shooter implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx shooterMotorTwo;
    public DcMotorEx shooterMotorOne;
    public ShooterState shooterState;
    public enum ShooterState {
        OFF,
        ON
    }

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;

        shooterMotorOne = map.get(DcMotorEx.class, "shooterMotorOne");
        shooterMotorTwo = map.get(DcMotorEx.class, "shooterMotorTwo");

        shooterMotorTwo.setDirection(DcMotorEx.Direction.REVERSE);

        shooterMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.shooterState = ShooterState.OFF;
    }


    @Override
    public void reset() {

    }

    @Override
    public void update() {
        switch (shooterState) {
            case OFF:
                shooterMotorOne.setPower(0);
                shooterMotorTwo.setPower(0);
                break;
            case ON:
                shooterMotorOne.setPower(0.5);
                shooterMotorTwo.setPower(0.5);
                break;
        }
    }

    @Override
    public String test() {
        return null;
    }


}
