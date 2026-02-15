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
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.Vec;

@Config
@TeleOp(name="Special Localization Test")
public class LocalizationTest extends LinearOpMode {
    public static boolean started = false;
    public static double speedSign = 1, headingSign = 1;
    public static double targetX = 24, targetY = 0, targetHeading = 0;
    public static double speedkP = .09, headingkP = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this, new Pose2d(0,0,0));

        waitForStart();

        while (opModeIsActive()) {

            robot.drive.updatePoseEstimate();
            if(gamepad1.bWasPressed())
                started = true;

            Pose2d pose = robot.drive.localizer.getPose();
            double headingError = MathUtils.angleNormDeltaRad(targetHeading - robot.drive.localizer.getPose().heading.toDouble());

//            robot.drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x
//                    ),
//                   !enableSimplePid ? -gamepad1.right_stick_x :
//                    Math.abs(headingError) < threshold ? 0 : kF * Math.signum(headingError)
//            ));
            if(started) {
                Vector2d targetDir = updateTargetDir(targetX, targetY, pose.position.x, pose.position.y, pose.heading.toDouble());
                double distanceError = Math.hypot(targetX - pose.position.x, targetY - pose.position.y);
                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                targetDir.x * speedkP * speedSign * distanceError,
                                targetDir.y * speedkP * distanceError
                        ),
                        headingError * headingkP * headingSign
                ));
                telemetry.addData("heading error", headingError);
                telemetry.addData("target dir x", targetDir.x);
                telemetry.addData("target dir y", targetDir.y);
                telemetry.update();
            }

            if(gamepad1.a)
                robot.drive.localizer.setPose(new Pose2d(0, 0, 0));

            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

    }
    private Vector2d updateTargetDir(double targetX, double targetY, double robotX, double robotY, double headingRad) {
        // translating target so that drivetrain is around origin
        double xFromRobot = targetX - robotX; // 48
        double yFromRobot = targetY - robotY; // 0
        // rotating target around origin
        double rotatedXFromRobot = xFromRobot * Math.cos(-headingRad) - yFromRobot * Math.sin(-headingRad);
        double rotatedYFromRobot = xFromRobot * Math.sin(-headingRad) + yFromRobot * Math.cos(-headingRad);
        //
        // translating target back to absolute; this returns the direction to the next waypoint IN THE ROBOT'S COORDINATE PLANE
        Vector2d targetDir = new Vector2d(rotatedXFromRobot, rotatedYFromRobot);
        double targetDirMag = Math.hypot(targetDir.x, targetDir.y);
        return targetDir.div(targetDirMag); // normalize
    }
}
