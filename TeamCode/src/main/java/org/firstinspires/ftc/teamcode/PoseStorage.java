package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/*
This class allows the TeleOp and Autonomous to send and receive robot pose between each other.
 */
public class PoseStorage {
    public static Pose2d currentPose = new Pose2d();
}