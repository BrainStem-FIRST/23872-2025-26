package org.firstinspires.ftc.teamcode.auto_subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.BrainSTEMAutoRobot;

import tele_subsystems.Collector;

public class AutoActions {

    public Action setCollect1(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT1;
                return false;
            }
        };
    }

    public Action setCollectorOn(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.collector.collectorState = Collector.CollectorState.ON;
                return false;
            }
        };
    }

    public Action robotUpdate(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.update();
                return false;
            }
        };
    }

    public Action setCollect2(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT2;
                return false;
            }
        };
    }

    public Action setCollect3(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT3;
                return false;
            }
        };
    }
}
