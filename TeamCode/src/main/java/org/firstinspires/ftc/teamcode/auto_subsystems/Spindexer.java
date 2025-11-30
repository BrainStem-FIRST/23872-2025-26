package org.firstinspires.ftc.teamcode.auto_subsystems;

//import static subsystems.Spindexer.spindexerState.OFF;
//import static subsystems.Spindexer.spindexerState.ON;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;


@Config
public class Spindexer implements Component {


    private int spindexerTargetPosition;
    public static double SPINDEXER_TICKS_PER_REVOLUTION = 288;

    public static double SPINDEXER_GEAR_RATIO = 1.0;

    public static final double SPINDEXER_TICKS_PER_DEGREE = (SPINDEXER_TICKS_PER_REVOLUTION * SPINDEXER_GEAR_RATIO) / 360.0;

    private static int COLLECT1_POS = 120;
    private static int COLLECT2_POS = 240;
    private static int COLLECT3_POS = 0;
    private static int SHOOT1_POS = 60;
    private static int SHOOT2_POS = 180;
    private static int SHOOT3_POS = 300;

    private int ticks;




    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx spindexerMotor;
    public SpindexerState spindexerState;
    public enum SpindexerState {
        COLLECT1,
        COLLECT2,
        COLLECT3,
        SHOOT1,
        SHOOT2,
        SHOOT3
    }

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;

        spindexerMotor = map.get(DcMotorEx.class, "spindexerMotor");

        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        spindexerMotor.setTargetPositionTolerance(2);

        spindexerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.spindexerState = SpindexerState.SHOOT1;

        spindexerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


    }

    public void rotateDegrees(double degrees){

        spindexerTargetPosition = spindexerMotor.getCurrentPosition() + (int)(degrees * SPINDEXER_TICKS_PER_DEGREE);

        spindexerMotor.setTargetPosition(spindexerTargetPosition);

        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spindexerMotor.setPower(0.2);

    }

    private int convertDegreesToTicks(double degrees){
        return (int)(degrees * SPINDEXER_TICKS_PER_DEGREE);
    }

    private void setTargetPosition(int targetPosition){
        spindexerMotor.setTargetPosition(targetPosition);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(1.0);
    }


    @Override
    public void reset() {

    }

    @Override
    public void update() {
        switch (spindexerState) {
            case COLLECT1:
                setTargetPosition(convertDegreesToTicks(COLLECT1_POS));
                break;
            case COLLECT2:
                setTargetPosition(convertDegreesToTicks(COLLECT2_POS));
                break;
            case COLLECT3:
                setTargetPosition(convertDegreesToTicks(COLLECT3_POS));
                break;
            case SHOOT1:
                setTargetPosition(convertDegreesToTicks(SHOOT1_POS));
                break;
            case SHOOT2:
                setTargetPosition(convertDegreesToTicks(SHOOT2_POS));
                break;
            case SHOOT3:
                setTargetPosition(convertDegreesToTicks(SHOOT3_POS));
                break;
        }
    }

    public boolean isSpindexerBusy(){
        return spindexerMotor.isBusy();
    }


@Override
public String test() {
    return null;
}


}

