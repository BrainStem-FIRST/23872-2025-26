package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;


@Config
public class Finger implements Component {

    private Telemetry telemetry;
    public Servo fingerServo;

    public FingerState fingerState;

    public HardwareMap map;
    public enum FingerState {
        DOWN,
        UP
    }

    public Finger(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.fingerState = FingerState.DOWN;

        fingerServo = map.get(Servo.class, "fingerServo");


    }
    @Override
    public void reset() {

    }

    @Override
    public void update() {
        switch (fingerState) {
            case DOWN:
                fingerServo.setPosition(0.0);
                break;
            case UP:
                fingerServo.setPosition(1.0);
                break;
        }
    }

    @Override
    public String test() {
        return null;
    }

}





