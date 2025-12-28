package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.tele_subsystems.Finger;
import org.firstinspires.ftc.teamcode.tele_subsystems.Shooter;
import org.firstinspires.ftc.teamcode.tele_subsystems.Spindexer;

public class ShootThreeBalls {

    private Shooter shooter;
    private Finger finger;
    private Spindexer spindexer;
    private Telemetry telemetry;

    public enum ShootThreeState{
        IDLE,
        SPIN_SHOOTER_AND_INDEXER,
        WAIT_FOR_SPINDEXER_TO_END,
        LIFT,
        LOWER,
        RESET,
        STOP
    }

    private ShootThreeState currentState = ShootThreeState.IDLE;
    private ElapsedTime stageTimer = new ElapsedTime();
    //^ tracks time so indexer+finger can be sequenced
    public ShootThreeBalls(Shooter shooterSubsystem, Finger fingerSubsystem, Spindexer spindexerSubsystem, Telemetry telemetry) {
        this.shooter = shooterSubsystem;
        this.finger = fingerSubsystem;
        this.spindexer = spindexerSubsystem;
        this.telemetry = telemetry;
    }
    //useful numbers

    private final double SHOOTER_TARGET_VELOCITY = 4000;
    static final double SHOOTER_VELOCITY_TOLERANCE = 0.05;
    static final double FINGER_LIFT_TIME = 3000;
    static final double FINGER_LOWER_TIME = 3000;
    static final double SPINDEXER_LONGER_THAN_THIS_MEANS_THERES_A_PROBLEM_MS = 3000;

    //Counts the balls
    private int ballsLeftToShoot = 3;
    private int ballsShot = 0;
    public void start() {
        if (currentState == ShootThreeState.IDLE) {
            telemetry.addData("Sequence", "Starting - Spinning up shooter...");
            stageTimer.reset();
            ballsShot = 0;
            ballsLeftToShoot = 3;
            currentState = ShootThreeState.SPIN_SHOOTER_AND_INDEXER;
        }
    }

    public ShootThreeState getCurrentState() {
        return currentState;
    }

    public void update() {

        switch (currentState) {
            case IDLE:

                shooter.currentState = Shooter.ShooterState.OFF;
                finger.fingerState = Finger.FingerState.DOWN;
                break;


            case SPIN_SHOOTER_AND_INDEXER:

                shooter.setShooterShootFar();
                finger.fingerState = Finger.FingerState.DOWN;

                double currentVelocity = Double.parseDouble(String.valueOf(shooter.shooterMotorOne.getVelocity()));
                double tolerance = SHOOTER_TARGET_VELOCITY * SHOOTER_VELOCITY_TOLERANCE;

                telemetry.addData("Current Shooter Velocity", "%.1f / %.1f", currentVelocity, SHOOTER_TARGET_VELOCITY);

                if (Math.abs(SHOOTER_TARGET_VELOCITY - currentVelocity) < tolerance) {
                    telemetry.addData("Shoot 3 Sequence Stage:", "Shooter is spinning up...");

                    spindexer.adjustPosition(40);

                    currentState = ShootThreeState.WAIT_FOR_SPINDEXER_TO_END;
                    stageTimer.reset();
                }

                break;

            case WAIT_FOR_SPINDEXER_TO_END:
                shooter.setShooterShootFar();
                finger.fingerState = Finger.FingerState.DOWN;

                if (spindexer.isStatic()) {
                    telemetry.addData("Stage", "Ok. Spindexer is good. We are shooting.");
                    currentState = ShootThreeState.LIFT;
                    stageTimer.reset();
                } else if (stageTimer.milliseconds() > SPINDEXER_LONGER_THAN_THIS_MEANS_THERES_A_PROBLEM_MS) {
                    telemetry.addData("Stage", "Oopsies oopsies theres a problem!");
                    currentState = ShootThreeState.STOP;
                }

                break;

            case LIFT:
                shooter.setShooterShootFar();
                finger.fingerState = Finger.FingerState.UP;

                if (stageTimer.seconds() > 0.5) {
                    telemetry.addData("Stage", "SOOOooo did the ball make it?");
                    currentState = ShootThreeState.LOWER;
                    stageTimer.reset();
                }

                break;

            case LOWER:

                shooter.setShooterShootFar(); // Keep shooter on
                finger.fingerState = Finger.FingerState.DOWN; // Move finger back down

             if (stageTimer.seconds() > 0.5) {

                ballsShot++; // ball has been shot
                if (ballsShot == 3) {
                    telemetry.addData("Stage", "All %d balls shot. We are stopping.", ballsLeftToShoot);
                    spindexer.adjustPosition(40);
                    stageTimer.reset();
                    currentState = ShootThreeState.RESET;
                } else {

                    telemetry.addData("Stage", "Ball %d shot. We are spinning.", ballsShot);

                     spindexer.adjustPosition(80);

                     currentState = ShootThreeState. WAIT_FOR_SPINDEXER_TO_END;
                     stageTimer.reset();
                 }
             }

                break;

            case RESET:

                shooter.setShooterOff();
                finger.fingerState = Finger.FingerState.DOWN;
                if (spindexer.isStatic()){
                    telemetry.addData("Stage", "Okay we are good.");
                    currentState = ShootThreeState.STOP;
                    stageTimer.reset();
                } else if (stageTimer.milliseconds() > SPINDEXER_LONGER_THAN_THIS_MEANS_THERES_A_PROBLEM_MS) {
                    telemetry.addData("Stage", "Help spindexer.");
                    currentState = ShootThreeState.STOP;
                }

                break;

            case STOP:
                shooter.setShooterOff(); // Turn shooter off
                finger.fingerState = Finger.FingerState.DOWN;  // Keep finger down




                currentState = ShootThreeState.IDLE;

                break;
        }


            telemetry.addData("Shoot State", getCurrentState().toString());
            telemetry.addData("Balls Shot", "%d / %d", ballsShot, ballsLeftToShoot);

    }

}

