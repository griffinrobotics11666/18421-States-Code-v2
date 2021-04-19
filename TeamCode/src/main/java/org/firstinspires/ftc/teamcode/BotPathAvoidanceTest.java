
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "GVF Obstacle Avoidance Test", group = "Discopolus")
@Disabled
public class BotPathAvoidanceTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Bot drive = new Bot(hardwareMap);

        Path test = drive.pathBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)))
                .forward(24)
                .build();

        waitForStart();

//        drive.followPath(test, new Obstacle[] {
//            new Obstacle(new Vector2d(18, -6), 6)
//        });
    }
}
