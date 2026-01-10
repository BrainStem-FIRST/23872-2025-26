package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
                collectorMotor.setPower(0);
                break;
            case INTAKE:
                collectorMotor.setPower(0.95);
                break;
            case EXTAKE:
                collectorMotor.setPower(-0.8);
                break;
            case AUTO:
                collectorMotor.setPower(0.8);
        }
    }
    @Override
    public String test() {return null;}


}