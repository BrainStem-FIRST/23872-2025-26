package org.firstinspires.ftc.teamcode.auto_subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;


@Config
public class Finger implements Component {
    public static int downPWM = 837, upPWM = 1258;
    public static double downPosition = 0.01, upPosition = 0.99;
    private Telemetry telemetry;
    public ServoImplEx fingerServo;

    public FingerState fingerState;

    public HardwareMap map;
    public enum FingerState {
        DOWN,
        UP;

    }

    public Finger(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;
        fingerState = FingerState.DOWN;

        fingerServo = map.get(ServoImplEx.class, "fingerServo");
        fingerServo.setPwmRange(new PwmControl.PwmRange(downPWM, upPWM));

    }
    @Override
    public void reset() {

    }

    @Override
    public void update() {
        switch (fingerState) {
            case DOWN:
                fingerServo.setPosition(downPosition);
                break;
            case UP:
                fingerServo.setPosition(upPosition);
                break;
        }
    }

    @Override
    public String test() {
        return null;
    }

}





