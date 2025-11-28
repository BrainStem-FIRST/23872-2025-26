package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import subsystems.Finger;
import subsystems.Shooter;
import subsystems.Spindexer;

public class ShootThreeBalls {

    private Shooter shooter;
    private Finger finger;
    private Spindexer spindexer;
    private Telemetry telemetry;

    public enum ShootThreeState{
        IDLE,
        SPIN_SHOOTER,
        SPIN_INDEXER,
        LIFT,
        LOWER,
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

    private final double SHOOTER_TARGET_VELOCITY = 19200;
    static final double SHOOTER_VELOCITY_TOLERANCE = 0.05;
    static final double SHOOTER_LONGER_THAN_THIS_MEANS_ITS_JAMMED_MS = 3000;
    static final double SPINDEXER_LONGER_THAN_THIS_MEANS_ITS_JAMMED_MS = 3000;
    static final double FINGER_LIFT_TIME = 3000;
    static final double FINGER_LOWER_TIME = 3000;

    //Counts the balls
    private int ballsLeftToShoot = 3;
    private int ballsShot = 0;
    public void start() {
        // Only start if we are currently idle (prevents spamming)
        if (currentState == ShootThreeState.IDLE) {
            telemetry.addData("Sequence", "Starting! Spinning up shooter...");
            currentState = ShootThreeState.SPIN_SHOOTER;
            stageTimer.reset();
            ballsShot = 0;
        }
    }

    public ShootThreeState getCurrentState() {
        return currentState;
    }

    public void update() {

        switch (currentState) {
            case IDLE:

                shooter.shooterState = Shooter.ShooterState.OFF;
                finger.fingerState = Finger.FingerState.DOWN;
                break;

            case SPIN_SHOOTER:

                shooter.shooterState = Shooter.ShooterState.ON;
                finger.fingerState = Finger.FingerState.DOWN;

                double currentVelocity = shooter.shooterMotorOne.getVelocity();
                double tolerance = SHOOTER_TARGET_VELOCITY * SHOOTER_VELOCITY_TOLERANCE;

                telemetry.addData("Current Shooter Velocity", "%.1f / %.1f", currentVelocity, SHOOTER_TARGET_VELOCITY);

                if (Math.abs(SHOOTER_TARGET_VELOCITY - currentVelocity) < tolerance) {
                    telemetry.addData("Shoot 3 Sequence Stage:", "Shooter is spinning up...");

                    spindexer.rotateDegrees(60);

                    currentState = ShootThreeState.SPIN_INDEXER;
                    stageTimer.reset();
                } else if  (stageTimer.milliseconds() > SHOOTER_LONGER_THAN_THIS_MEANS_ITS_JAMMED_MS){
                    telemetry.addData("Shoot 3 Sequence Stage:", "Something wrong: ");
                    currentState = ShootThreeState.STOP;
                }

                break;

            // 8. ADD NEW STATE LOGIC
            case SPIN_INDEXER:
                shooter.shooterState = Shooter.ShooterState.ON;
                finger.fingerState = Finger.FingerState.DOWN;

                if (spindexer.spindexerState == Spindexer.SpindexerState.OFF) {
                    telemetry.addData("Sequence", "Spindexer rotated. Lifting finger...");
                    currentState = ShootThreeState.LIFT;
                    stageTimer.reset();
                } else if (stageTimer.milliseconds() > SPINDEXER_LONGER_THAN_THIS_MEANS_ITS_JAMMED_MS) {
                    telemetry.addData("Sequence", "Spindexer TIMEOUT. Aborting.");
                    currentState = ShootThreeState.STOP;
                }

                break;

            case LIFT:
                shooter.shooterState = Shooter.ShooterState.ON;
                finger.fingerState = Finger.FingerState.UP;

                if (stageTimer.milliseconds() > FINGER_LIFT_TIME) {
                    telemetry.addData("Sequence", "Ball shot. Retracting finger.");
                    currentState = ShootThreeState.LOWER;
                    stageTimer.reset();
                }

                break;

            case LOWER:

                shooter.shooterState = Shooter.ShooterState.ON; // Keep shooter on
                finger.fingerState = Finger.FingerState.DOWN; // Move finger back down

             if (stageTimer.milliseconds() > FINGER_LOWER_TIME) {

                ballsShot++; // ball has been shot
                if (ballsShot >= ballsLeftToShoot) {
                    telemetry.addData("Sequence", "All %d balls shot. Stopping.", ballsLeftToShoot);
                    currentState = ShootThreeState.STOP;
                } else {
                    telemetry.addData("Sequence", "Ball %d shot. Spinning for next ball.", ballsShot);

                    spindexer.rotateDegrees(60);
                    
                    currentState = ShootThreeState.SPIN_INDEXER;
                    stageTimer.reset();
                }
             }


                break;

            case STOP:
                shooter.shooterState = Shooter.ShooterState.OFF; // Turn shooter off
                finger.fingerState = Finger.FingerState.DOWN;  // Keep finger down

                currentState = ShootThreeState.IDLE;

                break;
        }

        telemetry.addData("Shoot State", getCurrentState().toString());
        telemetry.addData("Balls Shot", "%d / %d", ballsShot, ballsLeftToShoot);
    }

}

