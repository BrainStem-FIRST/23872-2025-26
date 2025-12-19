package org.firstinspires.ftc.teamcode.auto_subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;
import org.firstinspires.ftc.teamcode.auto_subsystems.Collector;



@Config
public class Collector implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx collectorMotor;
    public CollectorState collectorState;
    public enum CollectorState {

        OFF,
        ON
    }

    public Collector(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;

        collectorState = CollectorState.OFF;

        collectorState = Collector.CollectorState.OFF;

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
            case ON:
                collectorMotor.setPower(0.5);
                break;
        }
    }
    @Override
    public String test() {return null;}


}

