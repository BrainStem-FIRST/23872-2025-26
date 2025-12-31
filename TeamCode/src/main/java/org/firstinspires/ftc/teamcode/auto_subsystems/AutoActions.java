package org.firstinspires.ftc.teamcode.auto_subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.BrainSTEMAutoRobot;


public class AutoActions {



    public Action setCollectorOn(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.collector.collectorState = Collector.CollectorState.ON;
                return false;
            }
        };
    }

    public Action setCollectorOff(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.collector.collectorState = Collector.CollectorState.OFF;
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


    public Action shooterTurnOnFar(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.shooter.setShooterShootFar();
                return false;
            }
        };
    }

    public Action shooterTurnOnClose(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.shooter.setShooterShootClose();
                return false;
            }
        };
    }

    public Action shooterTurnOnIdle(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.shooter.setShooterIdle();
                return false;
            }
        };
    }

    public Action shooterTurnOff(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.shooter.setShooterOff();
                telemetryPacket.addLine("Shooter Off");
                return false;
            }
        };
    }


    public Action moveSpindexer120(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spindexer.adjustPosition(-83);
                return false;
            }
        };
    }


    public Action moveSpindexer60(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spindexer.adjustPosition(-42);
                return false;
            }
        };
    }



    public Action fingerServoU(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.finger.fingerState = Finger.FingerState.UP;
                telemetryPacket.addLine("finger Up");
                return false;
            }
        };
    }

    public Action fingerServoD(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.finger.fingerState = Finger.FingerState.DOWN;
                telemetryPacket.addLine("finger Down");
                return false;
            }
        };
    }




}
