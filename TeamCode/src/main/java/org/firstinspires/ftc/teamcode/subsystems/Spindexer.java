package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    public int SPINDEXER_TIME;

    private double previousVelocity = 0;
    private int lastGoodPosition = 0;
    public boolean isUnjamming = false;
    public ElapsedTime spindexerTimer;
    public ElapsedTime antijamTimer;

    public enum SpindexerState {
        COLLECT,
        OFF, SHOOT
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

        spindexerPid = new PIDController(
                Constants.SpindexerConstants.INDEXER_KP,
                0,
                Constants.SpindexerConstants.INDEXER_KI
        );
        spindexerState = SpindexerState.COLLECT;
        spindexerTimer = new ElapsedTime();
        antijamTimer = new ElapsedTime();
        spindexerTimer.startTime();
    }
    public int getCurrentPosition() {
        return spindexerMotor.getCurrentPosition();
    }
    public void updateIndexerPosition() {
        if(isStatic())
            spindexerMotor.setPower(0);
        else {
            double power = spindexerPid.update(spindexerMotor.getCurrentPosition());
            power += Math.signum(power) * Constants.SpindexerConstants.INDEXER_KF;
            power = Range.clip(power, -Constants.SpindexerConstants.MAX_POWER, Constants.SpindexerConstants.MAX_POWER);
            spindexerMotor.setPower(power);
        }
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

        if ((Math.abs(spindexerMotor.getVelocity()) < Constants.SpindexerConstants.MIN_VEL_TO_START_CHECKING) && (Math.abs(error) > Constants.SpindexerConstants.MIN_ERROR_TO_START_CHECKING)){
            isUnjamming = true;
        }
        else
            isUnjamming = false;

        if(!isUnjamming)
            antijamTimer.reset();

        updateIndexerPosition();
        if (spindexerTimer.milliseconds() > SPINDEXER_TIME) {
            updateIndexerPosition();
        } else spindexerMotor.setPower(0);

    }

    public double getError() {
        return Math.abs(spindexerMotor.getCurrentPosition() - spindexerPid.getTarget());
    }
    public boolean isStatic() {
        return getError() < Constants.SpindexerConstants.ERROR_THRESHOLD;
    }
    @Override
    public String test() {
        return null;
    }


}

