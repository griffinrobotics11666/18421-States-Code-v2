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
    private Pose2d initialPose = new Pose2d(-63, -28.25, Math.toRadians(180));
    private Pose2d shootPose = new Pose2d(-4, -36, Math.toRadians(-2));

    private static double triggerStart = 0.33;
    private static double triggerEnd = 0.45;
    private static double armDown = 0.94;
    private static double armBetween = 0.45;
    private static double armUp = 0;
    private static double handOpen = 0.49;
    private static double handClosed = 0.08;

    private int ringCount;

    private ElapsedTime Time = new ElapsedTime();

    private Bot drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Bot(hardwareMap);
        drive.usingVuforia = false;
        drive.setPoseEstimate(initialPose);
        drive.initVision();

        drive.Arm.setPosition(armUp);
        drive.Arm.setSpeed(0.3);
        drive.Hand.setPosition(handClosed);
        drive.Trigger.setPosition(triggerStart);

        //Most Trajectories initialized here
//        Trajectory followStack = drive.trajectoryBuilder(initialPose)
//                .splineToConstantHeading(new Vector2d(-45,-33),0)
//                .build();
        //A
        Trajectory A1 =  drive.trajectoryBuilder(initialPose, 0.0)
                .lineToConstantHeading(new Vector2d(10,-44))
                .build();
        Trajectory wobble2 = drive.trajectoryBuilder(A1.end())
                .splineToSplineHeading(new Pose2d(-34, -42, Math.toRadians(90)), Math.toRadians(-180))
                .build();
        Trajectory A1End = drive.trajectoryBuilder(wobble2.end())
                .lineToLinearHeading(new Pose2d(1, -42, Math.toRadians(-180)))
                .build();
        //B
        Trajectory B1 = drive.trajectoryBuilder(initialPose, 0.0)
                .splineTo(new Vector2d(-20, -20),0.0)
                .splineToSplineHeading(new Pose2d(25, -36, Math.toRadians(-90)), 0.0)
                .build();
        Trajectory B2 = drive.trajectoryBuilder(B1.end())
                .lineToSplineHeading(new Pose2d(-4, -36, Math.toRadians(-5)))
                .build();
        Trajectory B3 = drive.trajectoryBuilder(B2.end())
                .lineToConstantHeading(new Vector2d(-24, -36))
                .build();
        Trajectory B1End = drive.trajectoryBuilder(wobble2.end(), 0)
                .splineToSplineHeading(new Pose2d(19, -39, Math.toRadians(-90)), 0.0)
                .build();
        //C
        Trajectory C1 = drive.trajectoryBuilder(B3.end())
                .lineToSplineHeading(new Pose2d(48, -60, Math.toRadians(-90)))
                .build();
        Trajectory C2 = drive.trajectoryBuilder(initialPose, 0.0)
                .splineToSplineHeading(new Pose2d(-10, -18, Math.toRadians(-2)), 0)
                .splineToConstantHeading(new Vector2d(-4, -36), 0)
                .build();
        Trajectory C1End = drive.trajectoryBuilder(wobble2.end())
                .lineToSplineHeading(new Pose2d(40, -60, Math.toRadians(-90)))
                .build();

//        while(!isStarted()) {
//            drive.detectStarterStack(1);
//        }
        waitForStart();

        Time.reset();

//        for(double i = drive.Arm.getPosition(); i <= armDown; i+=0.0005){
//            drive.Arm.setPosition(i);
//        }

        //Detects the starter stack
        ringCount = drive.pipeline.getStarterStack();

        if(ringCount == 0) {
            //Wobble 1
            drive.Arm.setPosition(armBetween);
            drive.followTrajectory(A1);
            plop();
            //Wobble 2
            drive.followTrajectory(wobble2);
            pickup();
            drive.followTrajectory(A1End);
            plop();
            //Shooting
            drive.followTrajectory(goShoot());
            shoot(4, 2080);
            //Parking
            park();
        }
        if(ringCount == 1){
            //Wobble 1
            drive.Arm.setPosition(armBetween);
            drive.followTrajectory(B1);
            plop();
            //Shooting
            drive.followTrajectory(B2);
            shoot(4, 2080);
            //Picking Up
            drive.Intake.setPower(-0.6);
            drive.followTrajectory(B3);
            sleep(1000);
            drive.Intake.setPower(0);
            //Shooting
            shoot(2, 2120);
            //Wobble 2
            drive.followTrajectory(wobble2);
            pickup();
            drive.followTrajectory(B1End);
            plop();
            //Parking
            park();
        }
        if(ringCount == 4){
            //Shooting
            drive.followTrajectory(C2);
            shoot(4, 2080);
            //Picking Up
            drive.Intake.setPower(-0.6);
            drive.followTrajectory(B3);
            sleep(1000);
            drive.Intake.setPower(0);
            //Shooting
            shoot(4, 2120);
            //Wobble 1
            drive.followTrajectory(C1);
            plop();
            //Wobble 2
            drive.followTrajectory(wobble2);
            pickup();
            drive.Arm.setPosition(armUp);
            drive.followTrajectory(C1End);
            plop();
            //Parking
            park();
        }

        drive.deactivateVision();
    }

    private void shoot(int times, double speed){
        drive.Shooter.setVelocity(speed);
        sleep(1000);
        for(int i = 0; i<times; i++){
            drive.Trigger.setPosition(triggerEnd);
            sleep(400);
            drive.Trigger.setPosition(triggerStart);
            sleep(400);
        }
        drive.Shooter.setVelocity(0);
    }

    private void park(){
        Trajectory parkLine = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(11, drive.getPoseEstimate().getY()))
                .build();
        drive.followTrajectory(parkLine);
    }

    private void plop(){
        drive.Arm.setPosition(armDown);
        drive.Hand.setPosition(handClosed);
        sleep(600);
        drive.Arm.setPosition(armUp);
        drive.Hand.setPosition(handOpen);
        sleep(200);
    }

    private void pickup(){
        drive.Hand.setPosition(handOpen);
        drive.Arm.setPosition(armDown);
        sleep(1000);
        drive.Hand.setPosition(handClosed);
        sleep(500);
        drive.Arm.setPosition(armBetween);
    }

    private Trajectory goShoot(){
        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-4, -36, Math.toRadians(-2)))
                .build();
    }
}
