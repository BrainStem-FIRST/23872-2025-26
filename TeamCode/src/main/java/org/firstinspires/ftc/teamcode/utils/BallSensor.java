package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BallSensor {


    public DigitalChannel beamBreak;
    public NormalizedColorSensor colorSensor;
    private boolean lastBeamState = true;

    private ElapsedTime timer = new ElapsedTime();
    private boolean isWaitingForColor = false;
    private double delayTimeMs = 150; // CHANGE


    private boolean isIndexing = false;



    public BallSensor(HardwareMap hardwareMap) {
        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        colorSensor.setGain(10);
    }





    public String scanForNewBall() {

        if (isIndexing) {
            return null;
        }

        boolean currentBeamState = beamBreak.getState(); // note false is broken

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

    public String detectColor() {

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;

        double sum = red + green + blue;

        double rPercent = red/sum;
        double gPercent = green/sum;
        double bPercent = blue/sum;

        // TODO: copy brob's code

        if (green > red && green > blue) {
            return "GREEN";
        } else if (red > green || blue > green) {
            return "PURPLE";
        } else if ((red+green+blue) < 100) {
            return  "EMPTY";
        }
        return "EMPTY";
    }



    public void setIfIndexerIsMoving(boolean moving) {
        if (this.isIndexing && !moving) {
            this.lastBeamState = true;
            this.isWaitingForColor = false;
        }

        // Update the state
        this.isIndexing = moving;
    }


}