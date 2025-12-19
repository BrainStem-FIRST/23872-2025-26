package org.firstinspires.ftc.teamcode.auto_subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

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

    public Action shooterTurnOnFar(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.shooter.shooterState = Shooter.ShooterState.SHOOT_FAR;
                return false;
            }
        };
    }

    public Action shooterTurnOff(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.shooter.shooterState = Shooter.ShooterState.OFF;
                return false;
            }
        };
    }
    public Action shooterMotorTwo(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        };
    }

    public Action shooterMotorOneOff(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        };
    }



    public Action SHOOT1_POS(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spindexer.spindexerState = Spindexer.SpindexerState.SHOOT1;
                return false;
            }
        };
    }


    public Action SHOOT2_POS(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spindexer.spindexerState = Spindexer.SpindexerState.SHOOT2;
                return false;
            }
        };
    }

    public Action SHOOT3_POS(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spindexer.spindexerState = Spindexer.SpindexerState.SHOOT3;
                return false;
            }
        };
    }

    public Action fingerServoU(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.finger.fingerState = Finger.FingerState.UP;
                return false;
            }
        };
    }

    public Action fingerServoD(BrainSTEMAutoRobot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.finger.fingerState = Finger.FingerState.DOWN;
                return false;
            }
        };
    }


}
