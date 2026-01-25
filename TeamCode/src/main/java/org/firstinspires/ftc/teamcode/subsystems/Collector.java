package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Component;


@Config
public class Collector implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx collectorMotor;
    public CollectorState collectorState;
    public enum CollectorState {

        OFF,
        INTAKE,
        EXTAKE,
        AUTO
    }

    public Collector(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;

        this.collectorState = CollectorState.OFF;



        collectorMotor = map.get(DcMotorEx.class, "collectorMotor");
    }


    @Override
    public void reset() {

    }

    @Override
    public void update() {
        switch (collectorState) {
            case OFF:
                collectorMotor.setPower(Constants.CollectorConstants.OFF_POWER);
                break;
            case INTAKE:
                collectorMotor.setPower(Constants.CollectorConstants.INTAKE_POWER);
                break;
            case EXTAKE:
                collectorMotor.setPower(Constants.CollectorConstants.EXTAKE_POWER);
                break;
            case AUTO:
                collectorMotor.setPower(Constants.CollectorConstants.AUTO_POWER);
        }
    }
    @Override
    public String test() {return null;}


}