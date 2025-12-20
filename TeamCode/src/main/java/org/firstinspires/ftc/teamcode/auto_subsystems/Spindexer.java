package org.firstinspires.ftc.teamcode.auto_subsystems;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMAutoRobot;
import org.firstinspires.ftc.teamcode.util.Component;
import org.firstinspires.ftc.teamcode.util.PIDController;


@Config
public class Spindexer implements Component {
    public static double indexerKP = 0.05;
    public static double errorThreshold = 5;
    public static double normalRotateDeg = 120;
    public static double shootRotateDeg = 30;
    public static double SPINDEXER_TICKS_PER_REVOLUTION = 241;

    public enum SpindexerState {
        COLLECT,
        OFF, SHOOT
    }
    public SpindexerState spindexerState;

    public PIDController spindexerPid;
    private int spindexerTargetPosition;
    private DcMotorEx spindexerMotor;
    private int curPos;
    private HardwareMap map;
    private Telemetry telemetry;

    public boolean indexerCued;
    private BrainSTEMAutoRobot robot;
    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMAutoRobot robot) {
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.robot = robot;

        spindexerMotor = map.get(DcMotorEx.class, "spindexerMotor");
        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        spindexerPid = new PIDController(indexerKP, 0, 0);
        spindexerState = SpindexerState.COLLECT;
    }

    public int rotateDegrees(double degrees){
        spindexerTargetPosition = spindexerMotor.getCurrentPosition() + (int)(degrees / 360. * SPINDEXER_TICKS_PER_REVOLUTION);
        spindexerPid.reset();
        spindexerPid.setTarget(spindexerTargetPosition);
        spindexerMotor.setTargetPosition(spindexerTargetPosition);
        spindexerMotor.setTargetPositionTolerance(2);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(0.3);
        return 0;
    }
    public int getCurrentPosition() {
        return spindexerMotor.getCurrentPosition();
    }
    public void rotate120degrees(){
        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setTargetPosition(96);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(0.5);
        if (spindexerMotor.getCurrentPosition() == 96){
            spindexerState = spindexerState.OFF;
        }
    }


    @Override
    public void reset() {

    }

    @Override
    public void update() {
        if(indexerCued && robot.finger.fingerState.equals(robot.finger.fingerState.DOWN)) {
            rotateDegrees(normalRotateDeg);
            indexerCued = false;
        }
        curPos = spindexerMotor.getCurrentPosition();

        if(isStatic()) {
            spindexerMotor.setPower(0);
        }
        else {
            double power = spindexerPid.update(spindexerMotor.getCurrentPosition());
            spindexerMotor.setPower(-power);
        }

        telemetry.addData("Spindexer Power", spindexerMotor.getPower());
        telemetry.addData("Spindexer Position", spindexerMotor.getCurrentPosition());
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

