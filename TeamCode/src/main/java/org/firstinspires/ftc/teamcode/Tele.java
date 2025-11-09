package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import subsystems.Collector;
import subsystems.Shooter;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class Tele extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private BrainSTEMRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, this);


        waitForStart();


        while (!opModeIsActive()) {

            telemetry.addData("Opmode Status :", "Init");

            telemetry.update();
        }



        while (opModeIsActive()) {
            telemetry.update();

//            robot.update();

            if(gamepad1.a) {
//               robot.spindexer.spindexerState = Spindexer.SpindexerState.ON;
                robot.spindexer.spindexerMotor.setPower(0.2);
            } else {
//                robot.spindexer.spindexerState = Spindexer.SpindexerState.OFF;
                robot.spindexer.spindexerMotor.setPower(0);
            }

            if(gamepad1.b){
//                robot.collector.collectorState = Collector.CollectorState.ON;
                robot.collector.collectorMotor.setPower(0.8);
            }else{
//                robot.collector.collectorState = Collector.CollectorState.ON;
                robot.collector.collectorMotor.setPower(0);
            }

            if(gamepad1.x){
//                robot.shooter.shooterState = Shooter.ShooterState.ON;
                robot.shooter.shooterMotorOne.setPower(1);
                robot.shooter.shooterMotorTwo.setPower(1);
            } else {
//                robot.shooter.shooterState = Shooter.ShooterState.OFF;
                robot.shooter.shooterMotorOne.setPower(0);
                robot.shooter.shooterMotorTwo.setPower(0);
            }


            double y = -gamepad1.left_stick_y*0.4;
            double x = gamepad1.left_stick_x*0.4;
            double rx = gamepad1.right_stick_x*0.4;

            robot.frontLeft.setPower(y + x + rx);
            robot.backLeft.setPower(y - x + rx);
            robot.frontRight.setPower(y - x - rx);
            robot.backRight.setPower(y + x - rx);
            telemetry.addData("frontLeft", robot.frontLeft.getPower());
            telemetry.addData("frontRight", robot.frontRight.getPower());
            telemetry.addData("backLeft", robot.backLeft.getPower());
            telemetry.addData("backRight", robot.backRight.getPower());



            telemetry.addData("y-axis :", y);
            telemetry.addData("x-axis :", x);
            telemetry.addData("turn :", rx);
            //telemetry.update();


        }
    }
}