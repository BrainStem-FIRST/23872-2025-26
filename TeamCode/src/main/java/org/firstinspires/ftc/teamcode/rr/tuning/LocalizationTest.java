package org.firstinspires.ftc.teamcode.rr.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.Drawing;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.TankDrive;

@Config
@TeleOp(name="Special Localization Test")
public class LocalizationTest extends LinearOpMode {
    public static double kF = .03;
    public static double targetHeading = Math.toRadians(90), threshold = Math.toRadians(5);
    public static boolean enableSimplePid = true;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this, new Pose2d(0,0,0));

        waitForStart();

        while (opModeIsActive()) {
            double headingError = targetHeading - robot.drive.localizer.getPose().heading.toDouble();
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                   !enableSimplePid ? -gamepad1.right_stick_x :
                    Math.abs(headingError) < threshold ? 0 : kF * Math.signum(headingError)
            ));

            if(gamepad1.a)
                robot.drive.localizer.setPose(new Pose2d(0, 0, 0));

            robot.drive.updatePoseEstimate();

            Pose2d pose = robot.drive.localizer.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

    }
}
