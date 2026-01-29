package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BallSensor {

    private DigitalChannel beamBreak;
    private ColorSensor colorSensor;
    private boolean lastBeamState = true;

    private ElapsedTime timer = new ElapsedTime();
    private boolean isWaitingForColor = false;
    private double delayTimeMs = 150; // CHANGE

    public BallSensor(HardwareMap hardwareMap) {
        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);
    }



    public String scanForNewBall() {
        boolean currentBeamState = beamBreak.getState(); // false means broken

        if (lastBeamState && !currentBeamState) {
            isWaitingForColor = true;
            timer.reset();
        }
        lastBeamState = currentBeamState;

        if (isWaitingForColor) {
            if (timer.milliseconds() > delayTimeMs) {
                isWaitingForColor = false;
                return detectColor();
            }
        }

        return null;
    }

    private String detectColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (green > red && green > blue) {
            return "GREEN";
        } else if (red > green || blue > green) {
            return "PURPLE";
        } else if ((red+green+blue) < 100) {
            return  "NA";
        }
        return "NA";
    }
}