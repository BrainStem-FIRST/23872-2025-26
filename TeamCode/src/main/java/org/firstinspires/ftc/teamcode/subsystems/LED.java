package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Component;
@Config
public class LED implements Component {
    public ServoImplEx led;
    private Telemetry telemetry;
    public HardwareMap map;

    public BrainSTEMRobot robot;


    public static double GREEN_POS = 0.285;
    public static double PURPLE_POS = 0.555;
    public static double WHITE_POS = 0.703;
    public static double PINK_POS = 0.52;
    public static double BLUE_POS = 0.4;
    public static double RED_POS = 0.01;

    public enum LedState {
        PINK,
        BLUE,
        GREEN,
        PURPLE,
        WHITE,
        RED
    }

    public LedState ledState;



    public LED(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.robot = robot;

        led = map.get(ServoImplEx.class, "ledLight");

        this.ledState = LedState.PINK;


    }


    @Override
    public void reset() {

    }

    @Override
    public void update() {
        switch (ledState) {
            case PINK:
                led.setPosition(PINK_POS);
                break;
            case BLUE:
                led.setPosition(BLUE_POS);
                break;
            case GREEN:
                led.setPosition(GREEN_POS);
                break;
            case PURPLE:
                led.setPosition(PURPLE_POS);
                break;
            case WHITE:
                led.setPosition(WHITE_POS);
                break;
            case RED:
                led.setPosition(RED_POS);
                break;
        }

    }

    @Override
    public String test() {
        return "";
    }

    private void setGreen() {
       ledState = LedState.GREEN;
    }
    public void setPink() {
        ledState = LedState.PINK;;
    }
    public void setBlue() {
        ledState = LedState.BLUE;
    }
    public void setRed() {
        ledState = LedState.RED;
    }

    private void setPurple() {
        ledState = LedState.PURPLE;
    }

    public void setWhite() {
        ledState = LedState.WHITE;
    }
}
