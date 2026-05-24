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
    private ServoImplEx led; Telemetry telemetry; HardwareMap map; BrainSTEMRobot robot; LedState ledState;
    public static double GREEN_POS = 0.285, PURPLE_POS = 0.555, WHITE_POS = 0.703, PINK_POS = 0.52, BLUE_POS = 0.4, RED_POS = 0.01;
    private enum LedState {
        PINK,
        BLUE,
        GREEN,
        PURPLE,
        WHITE,
        RED
    }

    public LED(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.robot = robot;
        this.ledState = LedState.PINK;

        led = map.get(ServoImplEx.class, "ledLight");
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
    public void setPink() {ledState = LedState.PINK;;}
    public void setBlue() {
        ledState = LedState.BLUE;
    }
    public void setRed() {
        ledState = LedState.RED;
    }
    private void setPurple() {
        ledState = LedState.PURPLE;
    }
    public void setWhite() { ledState = LedState.WHITE;}
}
