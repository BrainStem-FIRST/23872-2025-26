package org.firstinspires.ftc.teamcode.auto_subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BrainSTEMAutoRobot;


public class AutoActions {

    public Action setIndex1(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT1;
                return false;
            }
        };
    }
    public Action rotate120(BrainSTEMAutoRobot robot) {
        return new Action() {

            private boolean alreadyPressed = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
               if (!alreadyPressed){
                   robot.spindexer.rotate120Degrees();
                   robot.spindexer.spindexerState = Spindexer.SpindexerState.NORMAL;
               }
                return robot.spindexer.spindexerMotor.isBusy();

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
                return true;
            }
        };
    }

    public Action setIndex2(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT2;
                return false;
            }
        };
    }

    public Action setIndex3(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spindexer.spindexerState = Spindexer.SpindexerState.COLLECT3;
                return false;
            }
        };
    }



}
