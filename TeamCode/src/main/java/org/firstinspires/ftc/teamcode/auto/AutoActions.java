package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
            robot.collector.collectorState = Collector.CollectorState.AUTO;
            return false;
        };
    }

    public static Action setCollectorOff() {
        return telemetryPacket -> {
            robot.collector.collectorState = Collector.CollectorState.OFF;
            return false;
        };
    }

    public static Action robotUpdate(Telemetry telemetry) {
        return telemetryPacket -> {
            robot.update();
            telemetry.addData("Finger state", robot.finger.fingerState);
            telemetry.addData("Finger flicker time", robot.finger.flickerTimer);
            Pose2d robotPose = robot.drive.localizer.getPose();
            BrainSTEMRobot.autoX = robotPose.position.x;
            BrainSTEMRobot.autoY = robotPose.position.y;
            BrainSTEMRobot.autoH = robotPose.heading.toDouble();
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
    public static Action waitForAccurateShooterVelocity() {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean first = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    first = false;
                    timer.reset();
                }

                if (timer.seconds() >= 4)
                    return false;

                double error1 = Math.abs(Math.abs(robot.shooter.shooterMotorOne.getVelocity()) - robot.shooter.shooterPID1.getTarget());
                double error2 = Math.abs(Math.abs(robot.shooter.shooterMotorTwo.getVelocity()) - robot.shooter.shooterPID2.getTarget());

                double threshold = 30;
                return (error1 > threshold || error2 > threshold);
            }
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
        return moveSpindexer(Spindexer.ticks120);
    }


    public static Action moveSpindexer60() {
        return moveSpindexer(Spindexer.ticks60);
    }
    private static Action moveSpindexer(int ticks) {
        return new Action() {
            boolean first = true;
            double maxTime = 2;
            final ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    robot.spindexer.setSpindexerTargetAdjustment(ticks);
                    timer.reset();
                    first = false;
                }

                if (timer.seconds() >= maxTime)
                    return false;

                TelemetryPacket packet = new TelemetryPacket();
                packet.addLine("move spindexer " + ticks);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                return !robot.spindexer.isStatic();
            }
        };
    }

    public static Action fingerServoU() {
        return new SequentialAction (
                telemetryPacket -> {
                    robot.finger.fingerState = Finger.FingerState.UP;
                    robot.finger.flickerTimer.reset();
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.addLine("finger servo up");
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
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
