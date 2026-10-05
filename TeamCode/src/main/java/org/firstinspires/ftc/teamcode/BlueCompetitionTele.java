package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "BLUE Competition Tele Yay")
public class BlueCompetitionTele extends CompetitionTele {
    @Override
    public void runOpMode() throws InterruptedException {
        red = false;
        super.runOpMode();
    }
}
