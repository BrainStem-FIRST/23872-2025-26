package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.Drawing;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.PIDController;

/*
GP1:
A: turn shooter on
B: autoalign to red
X: spindexer 120
Y: Shooting sequence
Left trigger: extake
Right triger: intake
DP up: pink
DP down: blue
DP right: red
Dp left: white

GP2:
Right bumper: park down
Left bumper: park up
A, B, C, & D: stops opmode
 */

@Config
@TeleOp(name = "hod")
public class HoodTestTele extends LinearOpMode {

    public static double pos;


    private Pivot hood;
    ElapsedTime thisJamTime;




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        // CompetitionTelerobot = new BrainSTEMRobot(hardwareMap, this.telemetry, this, new Pose2d(BrainSTEMRobot.autoX, BrainSTEMRobot.autoY, BrainSTEMRobot.autoH));

       hood = new Pivot(hardwareMap, telemetry, null);







        telemetry.addLine("Robot is Ready!");
        telemetry.update();

        waitForStart();



        while (!opModeIsActive()) {
            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();

            hood.setDualServoPosition(pos);
            hood.update();




        }
    }



}