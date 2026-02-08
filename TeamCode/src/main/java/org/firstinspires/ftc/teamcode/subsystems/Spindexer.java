package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.srs.SRSHub;
import org.firstinspires.ftc.teamcode.utils.Component;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.Constants;


@Config
public class Spindexer implements Component {
    public static double maxPowerErrorThreshold = 150, maxPower = 0.99;

    public int SPINDEXER_TIME;

    private double previousVelocity;
    private int lastGoodPosition;
    public boolean isUnjamming = false;
    public ElapsedTime spindexerTimer;
    public ElapsedTime antijamTimer;

    private boolean wasMoving = false;
    public boolean justFinishedMoving = false;

    public PIDController spindexerPid;
    public DcMotorEx spindexerMotor;
    private int curPos;
    private HardwareMap map;
    private Telemetry telemetry;

    private int rawEncoder, wrappedEncoder;

    private int offset, wrapAroundOffset;

    private SRSHub hub;
    private double error;

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
                Constants.spindexerConstants.INDEXER_KP,
                0,
                Constants.spindexerConstants.INDEXER_KD
        );
        spindexerTimer = new ElapsedTime();
        antijamTimer = new ElapsedTime();
        spindexerTimer.startTime();






        SRSHub.Config config = new SRSHub.Config();

        config.setEncoder(
                6,
                SRSHub.Encoder.PWM
        );



        RobotLog.clearGlobalWarningMsg();

        hub = hardwareMap.get(
                SRSHub.class,
                "SRSHub"
        );

        spindexerPid.setTarget(rawEncoder); // TODO: check if it fixes


        hub.init(config);
        while(!hub.ready());


        this.offset = -907;

    }
    public int getCurrentPosition() {
        return wrappedEncoder;
    }

//    public int getAdjustedPosition() {
/// /        int reversedRaw = 1024 - encoder;
/// /        int adjusted = (reversedRaw - offset) % 1024;
/// /        if (adjusted < 0) {
/// /            adjusted += 1024;
/// /        }
/// /
/// /        return adjusted;
//
//    }
    public void updateIndexerPosition() {
        double error = (spindexerPid.getTarget() - getCurrentPosition());

//        if (error<0) {
//            error += 1024;
//        }

        double power = spindexerPid.update(getCurrentPosition());
        if (Math.abs(error) > maxPowerErrorThreshold) {
            spindexerMotor.setPower(maxPower * Math.signum(power));
        } else {
            power += Math.signum(power) * Constants.spindexerConstants.INDEXER_KF;

            power = Range.clip(power, -Constants.spindexerConstants.MAX_POWER, Constants.spindexerConstants.MAX_POWER);

            if (isStatic()) {
                spindexerMotor.setPower(0);
            } else {
                spindexerMotor.setPower(power);
            }
        }


    }

//    int target;
    public void setTargetAdj(int adjust) {
//        target += adjust;
//        target %= 1024;
        spindexerPid.setTarget(spindexerPid.getTarget() + adjust);
    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {
        hub.update();
        double prevEncoder = rawEncoder;
        rawEncoder = 1024 - (hub.readEncoder(6).position + offset);
        double dif = rawEncoder - prevEncoder;
        if(Math.abs(dif) > 500) {
            wrapAroundOffset -= (int)(1024* Math.signum(dif) );
        }
        wrappedEncoder = rawEncoder + wrapAroundOffset;

        error = spindexerMotor.getCurrentPosition() - spindexerPid.getTarget();

//        if ((Math.abs(spindexerMotor.getVelocity()) < Constants.spindexerConstants.MIN_VEL_TO_START_CHECKING) && (Math.abs(error) > Constants.spindexerConstants.MIN_ERROR_TO_START_CHECKING)){
//            isUnjamming = true;
//        }
//        else
//            isUnjamming = false;
//
//        if(!isUnjamming)
//            antijamTimer.reset();

        updateIndexerPosition();


        boolean isCurrentlyMoving = !isStatic();
        justFinishedMoving = wasMoving && !isCurrentlyMoving;
        wasMoving = isCurrentlyMoving;
    }

    public boolean isJammed() {
        if ((Math.abs(error) > 30) && Math.abs(spindexerMotor.getVelocity()) < 5 ) {
            return true;
        }

        return false;
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

