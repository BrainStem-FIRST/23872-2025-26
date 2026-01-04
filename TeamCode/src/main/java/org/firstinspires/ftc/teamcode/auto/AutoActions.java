package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.subsystems.Finger;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;


public class AutoActions {


    private static BrainSTEMRobot robot = null;
    public static void setRobot(BrainSTEMRobot robot) {
        AutoActions.robot = robot;
    }
    public static Action setCollectorOn() {
        return telemetryPacket -> {
            robot.collector.collectorState = Collector.CollectorState.INTAKE;
            return false;
        };
    }

    public static Action setCollectorOff() {
        return telemetryPacket -> {
            robot.collector.collectorState = Collector.CollectorState.OFF;
            return false;
        };
    }

    public static Action robotUpdate() {
        return telemetryPacket -> {
            robot.update();
            return true;
        };
    }


    public static Action shooterTurnOnFar() {
        return telemetryPacket -> {
            robot.shooter.shooterState = Shooter.ShooterState.SHOOT_FAR;
            telemetryPacket.addLine("Shooter On");
            return false;
        };
    }

    public static Action shooterTurnOnClose() {
        return telemetryPacket -> {
            robot.shooter.shooterState = Shooter.ShooterState.SHOOT_CLOSE;
            telemetryPacket.addLine("Shooter On");
            return false;
        };
    }

    public static Action turnShooterOnIdle() {
        return telemetryPacket -> {
            robot.shooter.shooterState = Shooter.ShooterState.IDLE;
            telemetryPacket.addLine("Shooter Idle");
            return false;
        };
    }

    public static Action shooterTurnOff() {
        return telemetryPacket -> {
            robot.shooter.shooterState = Shooter.ShooterState.OFF;
            telemetryPacket.addLine("Shooter Off");
            return false;
        };
    }
//    public Action shooterMotorTwo(BrainSTEMAutoRobot robot) {
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                return false;
//            }
//        };
//    }

//    public Action shooterMotorOneOff(BrainSTEMAutoRobot robot) {
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                telemetryPacket.addLine("Shooter Off");
//                return false;
//
//            }
//        };
//    }BrainSTEMRobot





    public static Action moveSpindexer120() {
        return moveSpindexer(Spindexer.degrees120);
    }


    public static Action moveSpindexer60() {
        return moveSpindexer(Spindexer.degrees60);
    }
    private static Action moveSpindexer(int ticks) {
        return new Action() {
            boolean first = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    robot.spindexer.setSpindexerTargetAdjustment(ticks);
                    first = false;
                }
                telemetryPacket.addLine("indexer S3");
                return !robot.spindexer.isStatic();
            }
        };
    }

    public static Action fingerServoU() {
        return new SequentialAction (
                telemetryPacket -> {
                    robot.finger.fingerState = Finger.FingerState.UP;
                    robot.finger.flickerTimer.reset();
                    telemetryPacket.addLine("finger Up");
                    return false;
                },
                new SleepAction(Finger.upTime)
        );
    }

    public static Action fingerServoD() {
        return new SequentialAction (
                telemetryPacket -> {
                    robot.finger.fingerState = Finger.FingerState.DOWN;
                    telemetryPacket.addLine("finger Down");
                    return false;
                },
                new SleepAction(Finger.downTime)
        );
    }
}
