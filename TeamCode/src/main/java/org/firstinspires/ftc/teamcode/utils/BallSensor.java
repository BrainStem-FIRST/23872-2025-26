package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class BallSensor {


    public DigitalChannel beamBreak;
    public NormalizedColorSensor colorSensor;
    private boolean lastBeamState = true;

    private ElapsedTime timer = new ElapsedTime();
    private boolean isWaitingForColor = false;

    public static double delayTimeMs = 100;


    private boolean isIndexing = false;

    public static double greenBallMinG = 0.39, greenBallMaxG = 0.53, greenBallMinB = 0.20, greenBallMaxB = 0.38, greenBallMinR = 0.10, greenBallMaxR = 0.25;
    public static double purpleBallMinG = 0.25, purpleBallMaxG = 0.4, purpleBallMinB = 0.25, purpleBallMaxB = 0.6, purpleBallMinR = 0.24, purpleBallMaxR = 0.32;

        // CHANGE

    public double rPercent;
    public double bPercent;
    public double gPercent;

    public double alpha;


    private ElapsedTime settleTimer = new ElapsedTime();
    private boolean waitingForSettle = false;
    public static double settleDelayMs = 0; // wait this long to read color after spindexer



    public BallSensor(HardwareMap hardwareMap) {
        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        colorSensor.setGain(10);
    }





    public String scanForNewBall() {
        boolean currentBeamState = !beamBreak.getState();

        if (isIndexing) {
            lastBeamState = currentBeamState;
            return null;
        }

        if (lastBeamState && !currentBeamState) {
            lastBeamState = currentBeamState;
            if (timer.milliseconds() > delayTimeMs) {
                isWaitingForColor = false;
                return detectColor();
            }
        }

        lastBeamState = currentBeamState;
        return null;
    }

    public String detectColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;
        alpha = colors.alpha;

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