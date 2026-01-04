package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.util.Component;


@Config
public class Finger implements Component {
    public static int downPWM = 837, upPWM = 1258;
    public static double downPosition = 0.07, upPosition = 0.99;
    public static double upTime = 0.2, downTime = 0.2;
    private Telemetry telemetry;
    public ServoImplEx fingerServo;

    public FingerState fingerState;

    public double targetPosition;

    public HardwareMap map;
    public enum FingerState {
        DOWN,
        UP
    }

    public ElapsedTime flickerTimer;

    private Spindexer spindexer;

    public Finger(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.spindexer = robot.spindexer;
        this.fingerState = FingerState.DOWN;

        fingerServo = map.get(ServoImplEx.class, "fingerServo");
        fingerServo.setPwmRange(new PwmControl.PwmRange(downPWM, upPWM));

        flickerTimer = new ElapsedTime();
        targetPosition = downPosition;
    }
    @Override
    public void reset() {

    }

    public void setTargetPosition() {
        fingerServo.setPosition(targetPosition);

        if(flickerTimer.seconds() > 1 && fingerState == FingerState.UP)
            fingerState = FingerState.DOWN;

        if(fingerState == FingerState.DOWN)
            flickerTimer.reset();

    }

    @Override
    public void update() {
        switch (fingerState) {
            case DOWN:
                targetPosition = downPosition;
                break;
            case UP:
                targetPosition = upPosition;
                telemetry.addData("Finger Timer", flickerTimer.milliseconds());
                spindexer.spindexerTimer.reset();
                break;
        }
        setTargetPosition();

        telemetry.addData("Finger State", fingerState);

    }

    public void moveFingerSlowly(){

    }

    @Override
    public String test() {
        return null;
    }

}





