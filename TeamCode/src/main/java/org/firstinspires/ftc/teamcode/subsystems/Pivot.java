package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Component;


@Config
public class Pivot implements Component {

    private Servo leftServo;
    private Servo rightServo;

    public Pivot(HardwareMap hardwareMap, Telemetry telemetry){


        leftServo = hardwareMap.get(Servo.class, "leftPivot");
        rightServo = hardwareMap.get(Servo.class, "rightPivot");

        leftServo.setDirection(Servo.Direction.FORWARD); // change
        rightServo.setDirection(Servo.Direction.REVERSE); // change

    }

    public void setPosition(double targetL, double targetR) {
        double goodTarR = Range.clip(targetR, Constants.PivotConstants.RIGHT_DOWN_POS, Constants.PivotConstants.RIGHT_UP_POS);
        double goodTarL = Range.clip(targetL, Constants.PivotConstants.LEFT_DOWN_POS, Constants.PivotConstants.LefT_UP_POS);


        leftServo.setPosition(goodTarL);
        rightServo.setPosition(goodTarR);
    }

    public void sePivotShootClose(){ setPosition(0.25,0.25);} // change
    public void setPivotShootFar() { setPosition(0.75,0.75);} // change

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


