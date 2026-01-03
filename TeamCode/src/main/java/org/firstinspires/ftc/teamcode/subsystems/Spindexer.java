package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.util.Component;
import org.firstinspires.ftc.teamcode.util.GamepadTracker;
import org.firstinspires.ftc.teamcode.util.PIDController;


@Config
public class Spindexer implements Component {
    public static double indexerKP = 0.02;
    public static double errorThreshold = 5;
    public static int degrees120 = 80, degrees60 = 40;
    public static double MILLIAMPS_FOR_JAM = 500;

    public ElapsedTime spindexerTimer;
    private ElapsedTime antijamTimer;

    public enum SpindexerState {
        COLLECT,
        OFF, SHOOT
    }
    public SpindexerState spindexerState;

    public PIDController spindexerPid;
    private int spindexerTargetPosition;
    public int spindexerTargetAdjustment;
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

        spindexerPid = new PIDController(indexerKP, 0, 0);
        spindexerState = SpindexerState.COLLECT;
        spindexerTimer = new ElapsedTime();
        antijamTimer = new ElapsedTime();
        spindexerTimer.startTime();
    }
    public int getCurrentPosition() {
        return spindexerMotor.getCurrentPosition();
    }
    public void adjustPosition() {

        if (spindexerTargetAdjustment != 0){
            antijamTimer.reset();
        }

        spindexerTargetPosition += spindexerTargetAdjustment;
        spindexerTargetAdjustment = 0;

        if (spindexerMotor.getCurrent(CurrentUnit.MILLIAMPS) > MILLIAMPS_FOR_JAM && antijamTimer.milliseconds() > 500) {
            double currentpower = spindexerMotor.getPower();
            telemetry.addData("antijam running:", "true");


            if (currentpower > 0 && antijamTimer.milliseconds() < 250){
                spindexerMotor.setPower(-0.2);
            } else if (antijamTimer.milliseconds() < 250){
                spindexerMotor.setPower(0.2);
            } else {
                spindexerMotor.setPower(0.0);
            }
            telemetry.addData("antijam running timer:", antijamTimer.milliseconds());
            telemetry.addData("AntiJam Power", spindexerMotor.getPower());


        } else {
//            antijamTimer.reset();
            spindexerPid.setTarget(spindexerTargetPosition);
            spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(isStatic()) {
                spindexerMotor.setPower(0);
            }
            else {
                double power = spindexerPid.update(spindexerMotor.getCurrentPosition());
                spindexerMotor.setPower(power);
            }
        }
    }

    public void setSpindexerTargetAdjustment(int adjust) {
        spindexerTargetAdjustment = adjust;
    }

    public void resetTimer() {
        spindexerTimer.reset();
    }

    @Override
    public void reset() {

    }

   

    @Override
    public void update() {
        if(spindexerTimer.milliseconds() > 500) {
            adjustPosition();
//            indexerCued = false;
        }
        curPos = spindexerMotor.getCurrentPosition();

        telemetry.addData("Spindexer Power", spindexerMotor.getPower());
        telemetry.addData("Spindexer Position", spindexerMotor.getCurrentPosition());
        telemetry.addData("Spindexer Current", spindexerMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("antijam running timer:", antijamTimer.milliseconds());


    }

    public boolean isStatic() {
        return Math.abs(curPos - spindexerPid.getTarget()) < errorThreshold;
    }
    public double getMotorPos() {
        return curPos;
    }


    @Override
    public String test() {
        return null;
    }


}

