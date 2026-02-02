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

    public static double greenBallMinG = 0.425, greenBallMaxG = 0.6, greenBallMinB = 0.34, greenBallMaxB = 0.38, greenBallMinR = 0.1275, greenBallMaxR = 0.19;
    public static double purpleBallMinG = 0.25, purpleBallMaxG = 0.415, purpleBallMinB = 0.38, purpleBallMaxB = 0.5, purpleBallMinR = 0.215, purpleBallMaxR =0.275;

        // CHANGE

    public double rPercent;
    public double bPercent;
    public double gPercent;


    private ElapsedTime settleTimer = new ElapsedTime();
    private boolean waitingForSettle = false;
    private double settleDelayMs = 100; // wait this long to read color after spindexer



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

        rPercent = red/sum;
        gPercent = green/sum;
        bPercent = blue/sum;

        if (rPercent > greenBallMinR && rPercent < greenBallMaxR &&
                bPercent > greenBallMinB && bPercent < greenBallMaxB &&
                gPercent > greenBallMinG && gPercent < greenBallMaxG){
            return "GREEN";
        } else if (rPercent > purpleBallMinR && rPercent < purpleBallMaxR &&
                bPercent > purpleBallMinB && bPercent < purpleBallMaxB &&
                gPercent > purpleBallMinG && gPercent < purpleBallMaxG){
            return "PURPLE";
        }

        return "EMPTY";
    }

    public String checkColorAfterMovement() {
        if (!waitingForSettle) {
            return null;
        }

        if (settleTimer.milliseconds() > settleDelayMs) {
            waitingForSettle = false;
            return detectColor();
        }

        return null;
    }


    public void setIfIndexerIsMoving(boolean moving) {

        if (this.isIndexing && !moving) {
            this.lastBeamState = true;
            this.isWaitingForColor = false;



            waitingForSettle = true;
            settleTimer.reset();
        }

        this.isIndexing = moving;
    }



}