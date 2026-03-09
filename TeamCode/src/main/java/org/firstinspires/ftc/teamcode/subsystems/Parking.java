package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Component;


@Config
public class Parking implements Component {

    public static boolean activateLeft = true, activateRight = false;
    private Telemetry telemetry;
    public ServoImplEx parkServo;
    public ServoImplEx parkServo2;

    public ParkState parkState;

    public double targetPosition;

    public HardwareMap map;
    public enum ParkState {
        DOWN,
        UP
    }

    public Parking(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.parkState = ParkState.DOWN;



        parkServo = map.get(ServoImplEx.class, "parkingServo1");
        parkServo2 = map.get(ServoImplEx.class, "parkingServo2");
        parkServo.setPwmRange(new PwmControl.PwmRange(Constants.rampConstants.DOWN_PWM, Constants.rampConstants.UP_PWM)); // CHANGE\
        parkServo2.setPwmRange(new PwmControl.PwmRange(Constants.rampConstants.DOWN_PWM, Constants.rampConstants.UP_PWM)); // CHANGE
        targetPosition = Constants.parkConstants.DOWN_POSITION;
    }
    @Override
    public void reset() {

    }

    public void setTargetPosition() {
        if (activateLeft) parkServo.setPosition(targetPosition);

        if (activateRight) parkServo2.setPosition(targetPosition);

    }

    @Override
    public void update() {
        switch (parkState) {
            case DOWN:
                targetPosition = Constants.rampConstants.DOWN_POSITION;
                break;
            case UP:
                targetPosition = Constants.rampConstants.UP_POSITION;
                break;
        }
        setTargetPosition();

        telemetry.addData("Parking State", parkState);

    }

    public void setRampUp() { parkState = ParkState.UP;}
    public void setRampDown() { parkState = ParkState.DOWN;};

    public ParkState checkRampState() {
        return parkState;
    }
    public boolean isRampUp() {
        return parkState == ParkState.UP.UP;
    }



    @Override
    public String test() {
        return null;
    }

}



