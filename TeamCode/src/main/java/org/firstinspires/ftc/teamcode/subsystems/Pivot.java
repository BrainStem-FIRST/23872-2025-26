package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Component;


@Config

public class Pivot implements Component {
    public static boolean activateLeft = true, activateRight = true;
    private ServoImplEx leftServo;
    private ServoImplEx rightServo;
    public double position;

    public static double closePivot = 0.59;
    public static double farPivot = 0.2;


    public double newPos;
    private OneWShooter shooter;
    public static int leftLower = 651, leftHigher = 2409, rightLower = 2240, rightHigher = 474; // 2132

    public enum PivotState{
        CLOSE,
        FAR,
        ADJUSTING
    }
    public PivotState pivotState;

    public Pivot(HardwareMap hardwareMap, Telemetry telemetry, OneWShooter shooter){


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
    // ADJUST THIS IF HOOD MOVEMENT IS TOO LARGE
    public static double HOOD_ADJ_SHOT = -0.04; // Adjust based on testing


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
                position = closePivot;
                setDualServoPosition(closePivot);
                break;
            case FAR:
                position = farPivot;
                setDualServoPosition(farPivot);
                break;
            case ADJUSTING:
                setDualServoPosition(newPos);
                break;

        }
    }

    @Override
    public String test() {
        return "";
    }
}




