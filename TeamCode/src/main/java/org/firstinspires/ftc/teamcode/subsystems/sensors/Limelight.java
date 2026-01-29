package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.BallTracker;
import org.firstinspires.ftc.teamcode.utils.Component;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@Config
public class Limelight implements Component {
    public int pipeline = 1;
    private Limelight3A limelight;

    public BallTracker ballTracker;



    public static int feducialResult = -1;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(pipeline);
        limelight.start();


       ballTracker = new BallTracker();

    }


    @Override
    public void update() {
        updateObeliskColors();

    }

    public void updateObeliskColors() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (!result.getFiducialResults().isEmpty())
                feducialResult = result.getFiducialResults().get(0).getFiducialId();

            else {
                feducialResult = -1;
            }
        }
        else {
            feducialResult = -10;
        }

        if (result != null) {
//            telemetry.addData("Fiducial results", result.getFiducialResults());
//            telemetry.addData("Amount", result.getFiducialResults().size());
        }
        switch(feducialResult){
            case(21):
                ballTracker.targetMotif = ballTracker.motif3;
                break;
            case(22):
                ballTracker.targetMotif = ballTracker.motif2;
                break;
            case(23):
                ballTracker.targetMotif = ballTracker.motif1;
                break;
        }
    }

    @Override
    public void reset() {

    }
    @Override
    public String test() {return null;}

}