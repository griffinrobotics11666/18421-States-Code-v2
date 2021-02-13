package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Starter Stack Detection", group="Discopolus")
public class BotStackDetectionTest extends LinearOpMode {
    private String detectedStack;
    @Override
    public void runOpMode() {
        Bot bot = new Bot(hardwareMap);
        bot.initVision();
        while(!isStarted()) {
            detectedStack = bot.detectStarterStack(100);
            telemetry.addData("detection: ", detectedStack);
            telemetry.update();
        }
        bot.deactivateVision();
    }
}
