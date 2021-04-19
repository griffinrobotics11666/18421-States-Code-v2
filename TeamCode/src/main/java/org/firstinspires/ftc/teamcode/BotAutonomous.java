package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Autonomous", group = "Discopolus")
@Config
/**
 * Team 18421's Autonomous for the 2020-2021 Ultimate Goal season.
 */
public class BotAutonomous extends LinearOpMode {
    private static Pose2d initialPose = new Pose2d(-63, -25, Math.toRadians(0));

    private static double triggerStart = 0.34;
    private static double triggerEnd = 0.1;
    private static double armDown = 0.94;
    private static double armBetween = 0.45;
    private static double armUp = 0;
    private static double handOpen = 0.49;
    private static double handClosed = 0.08;

    private String ringCount;

    private Trajectory wobble1;
    private Trajectory wobble2End;
    private Trajectory wobble1End;
    private ElapsedTime Time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Bot drive = new Bot(hardwareMap);
        drive.usingVuforia = false;
        drive.setPoseEstimate(initialPose);

        drive.Arm.setPosition(armUp);
        drive.Arm.setSpeed(0.3);
        drive.Hand.setPosition(handClosed);
        drive.Trigger.setPosition(triggerStart);
        drive.Latch.setPosition(0.46);

        //Most Trajectories initialized here
        Trajectory followStack = drive.trajectoryBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-45,-33),0)
                .build();
        Trajectory A1 =  drive.trajectoryBuilder(followStack.end())
                .splineToSplineHeading(new Pose2d(12,-44, Math.toRadians(-180)), 0.0)
                .build();
        Trajectory wobble2 = drive.trajectoryBuilder(A1.end())
                .splineToSplineHeading(new Pose2d(-34, -42, Math.toRadians(90)), Math.toRadians(-180))
                .build();
        Trajectory A1End = drive.trajectoryBuilder(wobble2.end())
                .lineToLinearHeading(new Pose2d(3, -40, Math.toRadians(-180)))
                .build();
//        Trajectory A1End = drive.trajectoryBuilder(A1.end())
//                .lineToSplineHeading(new Pose2d(A1.end().getX()+6, A1.end().getY(), Math.toRadians(-180.0)))
//                .lineToConstantHeading(new Vector2d(A1.end().getX()+6, A1.end().getY()+6))
//                .build();
        Trajectory B1 = drive.trajectoryBuilder(followStack.end())
                .splineTo(new Vector2d(-23, -20),0.0)
                .splineToSplineHeading(new Pose2d(25, -36, Math.toRadians(-90)), 0.0)
                .build();
        Trajectory B1End = drive.trajectoryBuilder(B1.end(), 0)
                .lineToSplineHeading(new Pose2d(B1.end().getX()-6, B1.end().getY()+6, Math.toRadians(0.0)))
                .build();
        Trajectory C1 = drive.trajectoryBuilder(followStack.end(), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-37,-56), 0)
                .splineToSplineHeading(new Pose2d(50, -60, Math.toRadians(-90)), 0.0)
                .build();
        Trajectory C1End = drive.trajectoryBuilder(C1.end(), 0)
                .lineToSplineHeading(new Pose2d(C1.end().getX()-6, C1.end().getY()-6, 0.0))
                .build();

//        while(!isStarted()) {
//            drive.detectStarterStack(1);
//        }
        waitForStart();

        Time.reset();

//        for(double i = drive.Arm.getPosition(); i <= armDown; i+=0.0005){
//            drive.Arm.setPosition(i);
//        }
        drive.Arm.setPosition(armUp);

        //Moves to detect starter stack
        drive.followTrajectory(followStack);
        //Detects the starter stack
        ringCount = drive.detectStarterStack(2000);
        switch(ringCount){
            case "None": {
                wobble1 = A1;
                wobble2End = A1End;
//                wobble1End = A1End;
                break;
            }
            case "Single": {
                wobble1 = B1;
                wobble2End = A1End;
                break;
            }
            case "Quad": {
                wobble1 = C1;
                wobble2End = A1End;
                break;
            }
        }

        //Moves loaded wobble goal accordingly
        drive.Arm.setPosition(armBetween);
        drive.followTrajectory(wobble1);
//        for(double i = drive.Arm.getPosition(); i <= armDown; i+=0.0005){
//            drive.Arm.setPosition(i);
//        }
        drive.Arm.setPosition(armDown);
        drive.Hand.setPosition(handClosed);
        sleep(600);
        drive.Arm.setPosition(armUp);
        drive.Hand.setPosition(handOpen);

        if(ringCount == "None") {
            drive.followTrajectory(wobble2);
            drive.Arm.setPosition(armDown);
            sleep(1000);
            drive.Hand.setPosition(handClosed);
            sleep(500);
            drive.Arm.setPosition(armBetween);

            drive.followTrajectory(wobble2End);

            drive.Arm.setPosition(armDown);
            drive.Hand.setPosition(handClosed);
            sleep(600);
            drive.Arm.setPosition(
                    armUp);
            drive.Hand.setPosition(handOpen);
        }


        //Shoots 3 rings into High Goal
        Trajectory goShoot = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-4, -36, Math.toRadians(1)))
                .build();
        drive.followTrajectory(goShoot);
        drive.Shooter.setVelocity(500, AngleUnit.DEGREES);
        sleep(1000);
        for(int i = 0; i<3; i++){
            drive.Trigger.setPosition(triggerEnd);
            sleep(400);
            drive.Trigger.setPosition(triggerStart);
            sleep(400);
        }

        //TODO: Pick up (and shoot) starter stack?
        //TODO: Pick up second wobble goal?

        //Parks on the white line
        Trajectory parkLine = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(11, drive.getPoseEstimate().getY()))
                .build();
        drive.followTrajectory(parkLine);

        //Woohoo!! 56 points!!!
        drive.deactivateVision();
    }
}
