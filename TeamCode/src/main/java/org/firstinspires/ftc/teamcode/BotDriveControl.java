package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.Cycle;
import org.firstinspires.ftc.teamcode.util.GamepadEx;

@Config
@TeleOp(name="Drive Control", group="Discopolus")
/**
 * Team 18421's TeleOp for the 2020-2021 Ultimate Goal season.
 */
public class BotDriveControl extends LinearOpMode {
    //Controls
    /*
    A = Toggle Arm Position - Low,High,Middle
    B = Intake Toggle
    X = Shoot Button - Only works if shooter is on
    Y = Shooter Toggle

    Left Joystick = Drive
    Right Joystick = Turn

    Dpad Left = Reverse Feeder - Only works if feeder is off
    Dpad Down = Quit Path Following
    Dpad Up = Reset Position using Vuforia
    Dpad Right = Reset Heading to 0 (Straight towards the goals)

    Back = Toggle Field Centric Drive

    Left Bumper = Automatically drive to shoot at High Goal

    Left Trigger = Linear Slide down
    Right Trigger = Linear Slide up
     */
    private GamepadEx gamepad;

    private static double triggerStart = 0.34;
    private static double triggerEnd = 0.1;

    private Button fieldCentric = new Button(true);
    public static Pose2d highGoal = new Pose2d(-5,-36, Math.toRadians(0));
    public static Pose2d powerShot = new Pose2d(2, -18.5, Math.toRadians(0));
    private boolean arePowerShooting = false;
//    public static Pose2d middlePower = new Pose2d(2, -11.5, Math.toRadians(0));
//    public static Pose2d leftPower = new Pose2d(2, -5.5, Math.toRadians(0));

    private ElapsedTime shootingClock = new ElapsedTime();
    public static double shootingDelay = 300;
    public static double shootingCooldown = 300;
    private enum ShootingState{
        SHOOT,
        RESET,
        WAIT
    }
    private ShootingState shoot = ShootingState.SHOOT;
    private Button isShooting = new Button();
    private Button isFeeding = new Button();
    private Cycle wobbleMode = new Cycle(3);
    private boolean isBackPressed = false;
    private double linearSlideCoefficient = 1;
    private double shootSpeed = 1;

    private enum Mode {
        DRIVER_CONTROL,
        PATH_FOLLOWING
    }
    private Mode mode = Mode.DRIVER_CONTROL;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new GamepadEx(gamepad1);
        Bot drive = new Bot(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
        drive.telemetry.addTelemetry(telemetry);
        drive.usingVuforia = false;

        drive.initVision();
        drive.Arm.setPosition(0.99);
        drive.Trigger.setPosition(triggerStart);
        drive.Latch.setPosition(0.0);

        drive.telemetry.addData("Ready!", "");
        drive.telemetry.update();
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            Pose2d currentPose = drive.getPoseEstimate();
            switch(mode){
                case DRIVER_CONTROL: {
                    //Linear Slide Control (Linear Slide was removed - not necessary)
//                    drive.linearSlide.setPosition(0.5-(gamepad1.left_trigger*linearSlideCoefficient*0.5)+(gamepad1.right_trigger*linearSlideCoefficient*0.5));

                    //Wobble Cycle
                    switch(wobbleMode.getValue()){
                        case 1: {
                            drive.Arm.setPosition(0.99);
                            break;
                        }
                        case 2: {
                            drive.Arm.setPosition(0.4);
                            break;
                        }
                        case 3: {
                            drive.Arm.setPosition(0.7);
                            break;
                        }
                    }
                    drive.telemetry.addData("wobble state", wobbleMode.getValue());
                    if(gamepad.a.justPressed()){
                        wobbleMode.cycle();
                    }

                    //Intake Toggle
                    if(gamepad.y.justPressed()){
                        isFeeding.toggle();
                    }
                    if(isFeeding.isPressed()){
                        drive.Intake.setPower(-0.6);
                    }
                    else {
                        //Intake Reversal
                        if(gamepad.dpad_left.isPressed()){
                            drive.Intake.setPower(0.6);
                        }
                        else drive.Intake.setPower(0);
                    }


                    //Shooter Toggle
                    if(gamepad.b.justPressed()){
                        isShooting.toggle();
                    }
                    if(isShooting.isPressed()){
                        drive.Shooter.setVelocity(1.0*shootSpeed*360, AngleUnit.DEGREES);
                    }
                    else drive.Shooter.setVelocity(0);

                    //Shooting Code
                    switch(shoot){
                        case SHOOT: {
                            if(gamepad.x.getState() && isShooting.isPressed()){
                                shootingClock.reset();
                                drive.Trigger.setPosition(triggerEnd);
                                shoot = ShootingState.RESET;
                                break;
                            }
                        }
                        case RESET: {
                            if(shootingClock.milliseconds()>= shootingDelay){
                                shootingClock.reset();
                                drive.Trigger.setPosition(triggerStart);
                                shoot = ShootingState.WAIT;
                                break;
                            }
                        }
                        case WAIT: {
                            if(shootingClock.milliseconds()>= shootingCooldown){
                                shootingClock.reset();
                                shoot= ShootingState.SHOOT;
                                break;
                            }
                        }
                    }

                    //Chassis Drive Code
                    if(fieldCentric.getState()){
                        // Create a vector from the gamepad x/y inputs
                        // Then, rotate that vector by the inverse of that heading
                        Vector2d input = new Vector2d(
                                gamepad1.left_stick_x,
                                -gamepad1.left_stick_y
                        ).rotated(-drive.getPoseEstimate().getHeading());

                        // Pass in the rotated input + right stick value for rotation
                        // Rotation is not part of the rotated input thus must be passed in separately
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        input.getX(),
                                        input.getY(),
                                        -gamepad1.right_stick_x
                                )
                        );
                    }
                    else {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y,
                                        -gamepad1.left_stick_x,
                                        -gamepad1.right_stick_x
                                )
                        );
                    }

                    //Automatic aim code
                    if(gamepad.left_bumper.justPressed()){
                        Trajectory followGoal = drive.trajectoryBuilder(currentPose)
                                .lineToLinearHeading(highGoal)
                                .build();
                        drive.followTrajectoryAsync(followGoal);
                        mode = Mode.PATH_FOLLOWING;
                    }

                    //Auto PowerShot code
                    if(gamepad.right_bumper.justPressed() && !arePowerShooting){
                        arePowerShooting = true;
                        shootSpeed = 0.8;
//                        Trajectory initialPos = drive.trajectoryBuilder(currentPose)
//                                .lineToLinearHeading(powerShot)
//                                .build();
//                        drive.followTrajectoryAsync(initialPos);
//                        mode = Mode.PATH_FOLLOWING;
                    }
                    else if(gamepad.right_bumper.justPressed() && arePowerShooting){
                        Trajectory adjust = drive.trajectoryBuilder(currentPose)
                                .strafeLeft(12)
                                .build();
                        drive.followTrajectoryAsync(adjust);
                        mode = Mode.PATH_FOLLOWING;
                    }
                    if(gamepad.left_stick_button.justPressed()){
                        arePowerShooting = false;
                        shootSpeed = 1;
                    }

                    //Reset Position
                    if(gamepad.dpad_up.isPressed()){
                        drive.usingVuforia = true;
                    }
                    else {
                        drive.usingVuforia = false;
                    }

                    //Reset Heading
                    if(gamepad.dpad_right.justPressed()){
                        drive.resetHeading();
                    }

                    //Field Centric Drive Toggle
                    if(gamepad.back.justPressed()) {
                        fieldCentric.toggle();
                    }

                    break;
                }
                case PATH_FOLLOWING: {
                    if(gamepad.dpad_down.justPressed()){
                        drive.forceIdle();
                        mode = Mode.DRIVER_CONTROL;
                    }
                    if(!drive.isBusy()){
                        mode = Mode.DRIVER_CONTROL;
                    }
                    break;
                }

            }

            //Update Cycle
            gamepad.update();
            drive.update();
        }
        drive.deactivateVision();
    }
}

