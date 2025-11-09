package subsystems;

//import static subsystems.Spindexer.spindexerState.OFF;
//import static subsystems.Spindexer.spindexerState.ON;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;


@Config
public class Spindexer implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx spindexerMotor;
    public SpindexerState spindexerState;
    public enum SpindexerState {
        OFF,
        ON
    }

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;

        spindexerMotor = map.get(DcMotorEx.class, "spindexerMotor");
    }


    @Override
    public void reset() {

    }

    @Override
    public void update() {
        switch (spindexerState) {
            case OFF:
                spindexerMotor.setPower(0);
                break;
            case ON:
                spindexerMotor.setPower(0.1);
                break;
        }
    }

    @Override
    public String test() {
        return null;
    }


}
