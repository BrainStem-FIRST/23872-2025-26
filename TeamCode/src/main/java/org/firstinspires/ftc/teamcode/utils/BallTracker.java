package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class BallTracker {

    //only tracks state - to be tested
    //declarations
    private Spindexer spindexer;
    private Telemetry telemetry;

    private static final int TOTAL_POSITONS = 6;

    public enum SpindexerMode{
        COLLECT, //evens: 0, 2, 4
        SHOOT// odds: 1, 3, 5
    }

    public enum BallColor {
        PURPLE,
        GREEN,
        EMPTY
    }
    private ArrayList<BallColor> positionsBall;
    private Queue<BallColor> ballColorsInQueue;
    private SpindexerMode currentMode = SpindexerMode.COLLECT;

    private static final int COLLECT_POSITION_1_LOADING = 0;
    private static final int COLLECT_POSITION_2 = 2;
    private static final int COLLECT_POSITION_3 = 4;

    private static final int SHOOT_POSITION_1 = 1;
    private static final int SHOOT_POSITION_2_SHOOTING = 3;
    private static final int SHOOT_POSITION_3 = 5;

    private int totalBallsInSpin = 0;
    private int totalBallsShot = 0;
    private int greenBallsInSpin = 0;
    private int purpleBallsInSpin = 0;
    private int purpleBallsShot = 0;
    private int greenBallsShot= 0;
    private int ballsQueued = 0;
    private static final int BALLS_PER_CYCLE = 3;


    public BallTracker(Telemetry telemetry){
        this.telemetry = telemetry;
        this.positionsBall = new ArrayList<>();
        this.ballColorsInQueue = new LinkedList<>();

        for (int i = 0; i < TOTAL_POSITONS; i++) {
            positionsBall.add(BallColor.EMPTY);
        }
    }

    public boolean addColorToQueau(BallColor color){
        if (color == BallColor.EMPTY){
            return false;
        }
        if (ballsQueued >= BALLS_PER_CYCLE){
            return false;
        }

        ballColorsInQueue.add(color);
        ballsQueued++;
        return true;
    }
    public void clearQueau() {
        ballColorsInQueue.clear();
        ballsQueued = 0;
    }

    public boolean isThereSpace() {
        return currentMode  == SpindexerMode.COLLECT && positionsBall.get(COLLECT_POSITION_1_LOADING) == BallColor.EMPTY;
    }

    public boolean loadNextBall() {
        if (!isThereSpace()) {
            return false;
        }
        if (ballColorsInQueue.isEmpty()){
            ballsQueued = 0;
            return false;
        }
        BallColor color = ballColorsInQueue.poll();
        positionsBall.set(COLLECT_POSITION_1_LOADING, color);
        totalBallsInSpin++;
        if(color == BallColor.GREEN) {
            greenBallsInSpin++;
        }
        if (color == BallColor.PURPLE){
            purpleBallsInSpin++;
        }

        return true;
    }

    public void rotated120() {
        BallColor first = positionsBall.remove(0);
        BallColor second = positionsBall.remove(0);
        positionsBall.add(first);
        positionsBall.add(second);
    }
    public void rotated60() {
        BallColor first = positionsBall.remove(0);
        positionsBall.add(first);

    }

    public void transToOtherMode(){
        if (currentMode == SpindexerMode.SHOOT){
            BallColor it = positionsBall.remove(0);
            positionsBall.add(it);
            currentMode = SpindexerMode.COLLECT;
        }
        if (currentMode == SpindexerMode.COLLECT){
            BallColor last = positionsBall.remove(positionsBall.size( ) - 1);
            positionsBall.add(0, last);
            currentMode = SpindexerMode.SHOOT;

        }
    }

    public BallColor recordShot() {
        if (currentMode != SpindexerMode.SHOOT) {
            return BallColor.EMPTY;
        }
        BallColor shot = positionsBall.get(SHOOT_POSITION_2_SHOOTING);
        if (shot == BallColor.EMPTY) {
            return BallColor.EMPTY;
        }

        positionsBall.set(SHOOT_POSITION_2_SHOOTING, BallColor.EMPTY);
        totalBallsShot++;
        totalBallsInSpin--;
        if (shot == BallColor.GREEN) {
            greenBallsShot++;
            greenBallsInSpin--;
        } else if (shot == BallColor.PURPLE) {
            purpleBallsShot++;
            purpleBallsInSpin--;
        }

        return shot;

    }
    public boolean canShoot(){
        return currentMode==SpindexerMode.COLLECT && !(positionsBall.get(3) == BallColor.EMPTY);
    }
    public boolean canCollect(){
        return currentMode==SpindexerMode.SHOOT && (positionsBall.get(0) == BallColor.EMPTY);
    }





}
