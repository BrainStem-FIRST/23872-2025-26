package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Component;


@Config

public class Pivot implements Component {
    private static boolean activateLeft = true, activateRight = true;
    private ServoImplEx leftServo, rightServo; Shooter shooter; PivotState pivotState;
    private static double closePivot = 0.62, pointPivot = 0.55, farPivot = 0.22, position, testingPosition = 0.6, newPos; // lower means its up more
    private static int leftLower = 651, leftHigher = 2409, rightLower = 2240, rightHigher = 474; // 2132
    private enum PivotState{
        CLOSE,
        FAR,
        AUTO,
        POINT,
        ADJUSTING
    }
    public double closeTargetPosition;

    public Pivot(HardwareMap hardwareMap, Telemetry telemetry, Shooter shooter){
        leftServo = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightHood");

        leftServo.setPwmRange(new PwmControl.PwmRange(leftLower, leftHigher));
        rightServo.setPwmRange(new PwmControl.PwmRange(rightLower, rightHigher));

        pivotState = PivotState.CLOSE;
        this.shooter = shooter;
    }

    public void setDualServoPosition(double position) {
        if(activateLeft)
            leftServo.setPosition(position);
        if(activateRight)
            rightServo.setPosition(position);
    }

    public static double HOOD_ADJ_SHOT = -0.04; // adjustment after every shot taken in far

    public void updateCompensatedPosition(int shotCount) {
        if (shotCount == 0) {return;}
        double basePos = position;
        newPos = basePos - (shotCount * HOOD_ADJ_SHOT);
        pivotState = PivotState.ADJUSTING;
    }
    public double getLeftPos() {
        return leftServo.getPosition();
    }
    public double getRightPos() {
        return rightServo.getPosition();
    }
    public void setPivotShootClose(){
        pivotState = PivotState.CLOSE;
    }
    public void setPivotShootPoint(){
        pivotState = PivotState.POINT;
    }

    public void setPivotShootAuto(){
        pivotState = PivotState.AUTO;
    }

    public void setPivotShootFar() {
        pivotState = PivotState.FAR;
    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {
        switch (pivotState) {
            case CLOSE:
                position = closeTargetPosition;
                setDualServoPosition(position);
                break;
            case POINT:
                position = pointPivot;
                setDualServoPosition(position);
                break;
            case FAR:
                position = farPivot;
                setDualServoPosition(farPivot);
                break;
            case ADJUSTING:
                setDualServoPosition(newPos);
                break;
            case AUTO:
                position = closePivot;
                setDualServoPosition(position);
                break;
        }
    }

    @Override
    public String test() {
        return "";
    }
}




