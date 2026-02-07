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
    public static int leftLower = 2367, leftHigher = 483, rightLower = 533, rightHigher = 2450;

    public Pivot(HardwareMap hardwareMap, Telemetry telemetry){


        leftServo = hardwareMap.get(Servo.class, "leftHood");
        rightServo = hardwareMap.get(Servo.class, "rightHood");

        ServoImplEx leftServoEx = (ServoImplEx) leftServo;
        ServoImplEx rightServoEx = (ServoImplEx) rightServo;


        PwmControl.PwmRange axonRange = new PwmControl.PwmRange(leftLower, leftHigher);
        leftServoEx.setPwmRange(axonRange);
        rightServoEx.setPwmRange(new PwmControl.PwmRange(rightLower, rightHigher));



    }

    public void setDualServoPosition(double position) {

//        leftServo.setPosition(Range.clip(position + Constants.PivotConstants.DUAL_OFFSET, 0, 1));
        if(activateLeft)
            leftServo.setPosition(position);
        if(activateRight)
            rightServo.setPosition(position);

//        rightServo.setPosition(1.0 - position);

    }

    public static double HOOD_ADJ_SHOT = 0.05; // Adjust based on testing


    public void updateCompensatedPosition(int shotCount) {
        double basePos = 0.75;
        double newPos = basePos + (shotCount * HOOD_ADJ_SHOT);

        leftServo.setPosition(newPos);
        rightServo.setPosition(newPos);
    }
    public double getLeftPos() {
        return leftServo.getPosition();
    }
    public double getRightPos() {
        return rightServo.getPosition();
    }
    public void setPivotShootClose(){
        setDualServoPosition(0.25);
    }
    public void setPivotShootFar() {
        setDualServoPosition(0.75);
    } // change

    @Override
    public void reset() {

    }

    @Override
    public void update() {

    }

    @Override
    public String test() {
        return "";
    }
}




