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
import org.firstinspires.ftc.teamcode.util.Component;
import org.firstinspires.ftc.teamcode.util.PIDController;


@Config
public class Spindexer implements Component {
    public static double indexerKP = 0.019; // 0.02
    public static double indexerKD = 0;
    public static double indexerKF = 0.01;
    public static double errorThreshold = 5;
    public static int ticks120 =96, ticks60 = 48;
    public static double MILLIAMPS_FOR_JAM = 500;
    public static double maxPower = 0.6;

    //anti jam
    public static double MIN_VEL_TO_START_CHECKING = 5;
    public static double minTimeToStartChecking = 1;
    public static double unJamTime = 0.3;
    public static double minErrorToStartChecking = 30;

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

        spindexerPid = new PIDController(indexerKP, 0, indexerKD);
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
            power += Math.signum(power) * indexerKF;
            power = Range.clip(power, -maxPower, maxPower);
            spindexerMotor.setPower(power);
        }
        telemetry.addData("spindexer power", spindexerMotor.getPower());
    }

    public void setSpindexerTargetAdjustment(int adjust) {
        spindexerPid.setTarget(spindexerPid.getTarget() + adjust);
    }

    @Override
    public void reset() {

    }

//   public boolean isSpindexerJammed(){
//        if ()
//   }

    @Override
    public void update() {
        double error = spindexerMotor.getCurrentPosition() - spindexerPid.getTarget();

//        if (!isUnjamming) {
//            previousVelocity = spindexerMotor.getPower();
//        }

        if ((Math.abs(spindexerMotor.getVelocity()) < MIN_VEL_TO_START_CHECKING) && (Math.abs(error) > minErrorToStartChecking)){
            isUnjamming = true;
        }
        else
            isUnjamming = false;

        if(!isUnjamming)
            antijamTimer.reset();

//        if ((spindexerMotor.getVelocity() > MIN_VEL_TO_START_CHECKING) && (error < minErrorToStartChecking)){
//            isUnjamming = false;
//        }

//        if (isUnjamming && antijamTimer.seconds() > minTimeToStartChecking) {
//            if(antijamTimer.seconds() > unJamTime + minTimeToStartChecking)
//                isUnjamming = false;
//            else
//                spindexerMotor.setPower(-Math.signum(error) * 0.2);
//
//            return;
//        }
        updateIndexerPosition();
        if (spindexerTimer.milliseconds() > 500) {
            updateIndexerPosition();
        } else spindexerMotor.setPower(0);






        telemetry.addData("spindexer error", getError());
        telemetry.addData("is static", isStatic());
        telemetry.addData("Spindexer Power", spindexerMotor.getPower());
        telemetry.addData("Spindexer Position", spindexerMotor.getCurrentPosition());
        telemetry.addData("Spindexer Current", spindexerMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("antijam running timer:", antijamTimer.milliseconds());


    }

    public double getError() {
        return Math.abs(spindexerMotor.getCurrentPosition() - spindexerPid.getTarget());
    }
    public boolean isStatic() {
        return getError() < errorThreshold;
    }
    @Override
    public String test() {
        return null;
    }


}

