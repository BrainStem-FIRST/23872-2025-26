package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.Component;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.Constants;


@Config
public class Spindexer implements Component {

    public int SPINDEXER_TIME = 0;

    private double previousVelocity = 0;
    private int lastGoodPosition = 0;
    public boolean isUnjamming = false;
    public ElapsedTime spindexerTimer;
    public ElapsedTime antijamTimer;

    private boolean wasMoving = false;
    public boolean justFinishedMoving = false;

    public enum SpindexerState {
        COLLECT,
        MOVING,
        NOT_MOVING,
    }
    public SpindexerState spindexerState;

    public PIDController spindexerPid;
    public DcMotorEx spindexerMotor;
    private int curPos;
    private HardwareMap map;
    private Telemetry telemetry;


    public boolean indexerCued;
    private BrainSTEMRobot robot;
    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.robot = robot;



        spindexerMotor = map.get(DcMotorEx.class, "spindexerMotor");
        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        spindexerPid = new PIDController(
                Constants.spindexerConstants.INDEXER_KP,
                0,
                Constants.spindexerConstants.INDEXER_KD
        );
        spindexerState = SpindexerState.NOT_MOVING;
        spindexerTimer = new ElapsedTime();
        antijamTimer = new ElapsedTime();
        spindexerTimer.startTime();

        spindexerPid.setTarget(0);
    }
    public int getCurrentPosition() {
        return spindexerMotor.getCurrentPosition();
    }
    public double updateIndexerPosition() {
        double power = spindexerPid.update(spindexerMotor.getCurrentPosition());


            power += Math.signum(power) * Constants.spindexerConstants.INDEXER_KF;

        power = Range.clip(power, -Constants.spindexerConstants.MAX_POWER, Constants.spindexerConstants.MAX_POWER);

        if (isStatic()) {
            spindexerMotor.setPower(0);
        } else {
            spindexerMotor.setPower(-power);
        }

        return power;
    }

    public void setSpindexerTargetAdjustment(int adjust) {

        spindexerPid.setTarget(spindexerPid.getTarget() + adjust);
    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {
        double error = spindexerMotor.getCurrentPosition() - spindexerPid.getTarget();

        if ((Math.abs(spindexerMotor.getVelocity()) < Constants.spindexerConstants.MIN_VEL_TO_START_CHECKING) && (Math.abs(error) > Constants.spindexerConstants.MIN_ERROR_TO_START_CHECKING)){
            isUnjamming = true;
        }
        else
            isUnjamming = false;

        if(!isUnjamming)
            antijamTimer.reset();

        updateIndexerPosition();


        boolean isCurrentlyMoving = !isStatic();
        justFinishedMoving = wasMoving && !isCurrentlyMoving;
        wasMoving = isCurrentlyMoving;


        if (isStatic()) {
            spindexerState = SpindexerState.NOT_MOVING;
        } else {
            spindexerState = SpindexerState.MOVING;
        }
    }

    public double getError() {
        return Math.abs(spindexerMotor.getCurrentPosition() - spindexerPid.getTarget());
    }
    public boolean isStatic() {
        return getError() < Constants.spindexerConstants.ERROR_THRESHOLD;
    }
    @Override
    public String test() {
        return null;
    }


}

