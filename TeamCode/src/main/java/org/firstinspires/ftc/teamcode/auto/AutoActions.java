package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Collector;
import org.firstinspires.ftc.teamcode.subsystems.Finger;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.sensors.Limelight;


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
//            telemetry.addData("Finger state", robot.finger.fingerState);
//            telemetry.addData("Finger flicker time", robot.finger.flickerTimer);
            Pose2d robotPose = robot.drive.localizer.getPose();
            BrainSTEMRobot.autoX = robotPose.position.x;
            BrainSTEMRobot.autoY = robotPose.position.y;
            BrainSTEMRobot.autoH = robotPose.heading.toDouble();
            return true;
        };
    }




    // SHOOTER

    public static Action shooterTurnOnFar() {
        return telemetryPacket -> {
            robot.shooter.setShooterShootFar();
            telemetryPacket.addLine("Shooter On");
            return false;
        };
    }

    public static Action shooterTurnOnClose() {
        return telemetryPacket -> {
            robot.shooter.setShooterShootClose();
            telemetryPacket.addLine("Shooter On");
            return false;
        };
    }


    public static Action turnShooterOnIdle() {
        return telemetryPacket -> {
            robot.shooter.setShooterIdle();
            telemetryPacket.addLine("Shooter Idle");
            return false;
        };
    }

    public static Action shooterTurnOff() {
        return telemetryPacket -> {
            robot.shooter.setShooterOff();
            telemetryPacket.addLine("Shooter Off");
            return false;
        };
    }

    // SPINDEXER

    public static Action moveSpindexer120() {
        robot.limelight.ballTracker.rotated120();
        return moveSpindexer(Constants.SpindexerConstants.TICKS_120);
    }


    public static Action moveSpindexer60() {
        robot.limelight.ballTracker.rotated60();
        return moveSpindexer(Constants.SpindexerConstants.TICKS_60);
    }


    public static Action moveSpindexer360() {
        robot.limelight.ballTracker.rotated60();
        return moveSpindexer(Constants.SpindexerConstants.TICKS_360);
    }

    public static Action moveSpindexerALittle() {
        return moveSpindexer(8);
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

    // OLD FINGER
    public static Action fingerServoU() {
        return new SequentialAction (
                telemetryPacket -> {
//                    robot.finger.fingerState = Finger.FingerState.UP;
//                    robot.finger.flickerTimer.reset();
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.addLine("finger servo up");
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                    return false;
                },
                new SleepAction(Constants.FingerConstants.UP_TIME)
        );
    }


    public static Action fingerServoD() {
        return new SequentialAction (
                telemetryPacket -> {
//                    robot.finger.fingerState = Finger.FingerState.DOWN;
                    telemetryPacket.addLine("finger Down");
                    return false;
                },
                new SleepAction(Constants.FingerConstants.DOWN_TIME)
        );
    }

    // RAMP
    public static Action rampUp() {
        return new SequentialAction (
                telemetryPacket -> {
                    robot.ramp.setRampUp();
                    return false;
                }
        );
    }
    public static Action rampDown() {
        return new SequentialAction (
                telemetryPacket -> {
                    robot.ramp.setRampDown();
                    return false;
                }
        );
    }

    // PIVOT

    public static Action pivotClose() {
        return new SequentialAction (
                telemetryPacket -> {
                    robot.pivot.sePivotShootClose();
                    return false;
                }
        );
    }

    public static Action pivotFar() {
        return new SequentialAction (
                telemetryPacket -> {
                    robot.pivot.setPivotShootFar();
                    return false;
                }
        );
    }


    // OTHER

    public static Action shootAll() {
        return new SequentialAction (
                telemetryPacket -> {
                    toStartOfPatternShoot();
                    rampUp();
                    moveSpindexer360();
                    new SleepAction(0.5);
                    turnShooterOnIdle();
                    return false;
                }
        );
    }

    public static Action toStartOfPatternShoot() {
        return new Action() {
            boolean frist = true;
            final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (frist) {
                    int rotationAmount = robot.limelight.ballTracker.getBestRotation();

                    if (rotationAmount == 0) {
                        return false;
                    }

                    robot.spindexer.setSpindexerTargetAdjustment(rotationAmount);
                    timer.reset();
                    frist = false;

                }

                if (timer.seconds() >= 2.0) {
                    return false;
                }

                return !robot.spindexer.isStatic();
            }
        };
    }


    public Action triggerSpindexerAtPos(double targetY) {
        return new Action() {
            private boolean triggered = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!triggered && Math.abs(robot.drive.localizer.getPose().position.y - targetY) < 2.0) {
                    robot.spindexer.setSpindexerTargetAdjustment(96);
                    robot.limelight.ballTracker.rotated60();
                    triggered = true;
                }
                return !triggered;
            }
        };
    }

    // TODO: Fix wait for Accurate Shooter Vel
//    public static Action waitForAccurateShooterVelocity() {
//        return new Action() {
//            final ElapsedTime timer = new ElapsedTime();
//            boolean first = true;
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if (first) {
//                    first = false;
//                    timer.reset();
//                }
//
//                if (timer.seconds() >= 2)
//                    return false;
//
//                double error1 = Math.abs(Math.abs(robot.shooter.shooterMotorOne.getVelocity()) - robot.shooter.shooterPID1.getTarget()); // change
//                double error2 = Math.abs(Math.abs(robot.shooter.shooterMotorTwo.getVelocity()) - robot.shooter.shooterPID2.getTarget()); // change
//
//                double threshold = 30;
//                return (error1 > threshold || error2 > threshold);
//            }
//        };
//    }

    public static Action waitForAccurateShooterVelocity() {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean first = true;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    timer.reset();
                    first = false;
                }

                if (timer.seconds() >= 2.0) {
                    return false;
                }


                // 4. Debugging Telemetry
                telemetryPacket.put("Shooter Target", robot.shooter.targetVel);
                telemetryPacket.put("Shooter V1", robot.shooter.currentVel1);
                telemetryPacket.put("Shooter V2", robot.shooter.currentVel2);
                telemetryPacket.put("Error V1", robot.shooter.error1);

                // 5. Check Threshold
                double threshold = 30;

                // Return true (keep running) if error is too high
                return (robot.shooter.error1 > threshold || robot.shooter.error2 > threshold);
            }
        };
    }




}
