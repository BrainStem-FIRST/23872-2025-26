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
    private Servo leftServo;
    private Servo rightServo;
    public double position;

    public double newPos;
    private OneWShooter shooter;
    public static int leftLower = 2367, leftHigher = 483, rightLower = 533, rightHigher = 2450;

    public enum PivotState{
        CLOSE,
        FAR,
        ADJUSTING
    }
    public PivotState pivotState;

    public Pivot(HardwareMap hardwareMap, Telemetry telemetry, OneWShooter shooter){


        leftServo = hardwareMap.get(Servo.class, "leftHood");
        rightServo = hardwareMap.get(Servo.class, "rightHood");

        ServoImplEx leftServoEx = (ServoImplEx) leftServo;
        ServoImplEx rightServoEx = (ServoImplEx) rightServo;


        PwmControl.PwmRange axonRange = new PwmControl.PwmRange(leftLower, leftHigher);
        leftServoEx.setPwmRange(axonRange);
        rightServoEx.setPwmRange(new PwmControl.PwmRange(rightLower, rightHigher));

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
    public static double HOOD_ADJ_SHOT = -0.02; // Adjust based on testing


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
                position = 0.25;
                setDualServoPosition(0.25);
                break;
            case FAR:
                position = 0.4;
                setDualServoPosition(0.4);
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




