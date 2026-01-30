package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.BallTracker;
import org.firstinspires.ftc.teamcode.utils.BallTrackerNew;
import org.firstinspires.ftc.teamcode.utils.Component;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@Config
public class Limelight implements Component {
    public int pipeline = 0;
    private Limelight3A limelight;

    public BallTrackerNew ballTrackerNew;



    public static int feducialResult = -1;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(pipeline);
        limelight.start();


       ballTrackerNew = new BallTrackerNew(robot.spindexer);

    }


    @Override
    public void update() {
        ballTrackerNew.update();
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
                ballTrackerNew.targetMotif = ballTrackerNew.motif3;
                break;
            case(22):
                ballTrackerNew.targetMotif = ballTrackerNew.motif2;
                break;
            case(23):
                ballTrackerNew.targetMotif = ballTrackerNew.motif1;
                break;
        }
    }

    @Override
    public void reset() {

    }
    @Override
    public String test() {return null;}

}