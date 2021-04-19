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
    A = Open/Close Hand
    B =
    X = Shoot Button - Only works if shooter is on
    Y = Intake Toggle

    Left Joystick = Drive
    Right Joystick = Turn

    Dpad Up = Reverse Feeder - If the feeder is on it will turn it off first
    Dpad Down = Quit Path Following
    Dpad Left =
    Dpad Right =

    Back = Reset Heading to 0 (Straight towards the goals)
    Start = Toggle Field Centric Drive

    Right Stick Button = Reset Position to the top left
    Left Stick Button = Reset Powershot following

    Left Trigger = Toggle Arm Position between Low, Middle, and High
    Right Trigger = Turn on Shooter

    Right Bumper = Automatically drive to shoot at High Goal
    Left Bumper = Automatically drive to shoot at Powershot Targets
     */
    private GamepadEx gamepad;

    private static double triggerStart = 0.3;
    private static double triggerEnd = 0.6;

    private Button fieldCentric = new Button(true);
    public static Pose2d highGoal = new Pose2d(-3,-36, Math.toRadians(0));
    public static Pose2d powerShot = new Pose2d(-3, -18.5, Math.toRadians(0));
    public static Pose2d wobbleDrop = new Pose2d(-60, -23.25, Math.toRadians(90));
    private boolean arePowerShooting = false;
//    public static Pose2d middlePower = new Pose2d(2, -11.5, Math.toRadians(0));
//    public static Pose2d leftPower = new Pose2d(2, -5.5, Math.toRadians(0));

    private ElapsedTime shootingClock = new ElapsedTime();
    private ElapsedTime reversalClock = new ElapsedTime();
    public static double shootingDelay = 300;
    public static double shootingCooldown = 300;
    private enum ShootingState{
        SHOOT,
        RESET,
        WAIT
    }
    private ShootingState shoot = ShootingState.SHOOT;
    public static double defaultSpeed = 2587;
    private Button isShooting = new Button();
    private Button isFeeding = new Button();
    public static double intakeReversalDelay = 500;
    private Button isReversing = new Button();
    private Button handState = new Button();
    private Cycle wobbleMode = new Cycle(3);
    private boolean isBackPressed = false;
    private double linearSlideCoefficient = 1;
    private double targetHeight = 35.8;
    private Vector2d targetPos = new Vector2d(70.75, -46.5+12);
    public static double highGoalHeight = 35.8;
    public static double powerShotHeight = 30.8;
    public static Vector2d highGoalPos = new Vector2d(70.75, -46.5+12);
    public static double shotDistance = 70.75;


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
        drive.usingVuforia = false;

        drive.Arm.setPosition(0);
        drive.Arm.setSpeed(1);

        drive.Hand.setPosition(0.45);
        drive.Trigger.setPosition(triggerStart);

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
                            drive.Arm.setPosition(0);
                            break;
                        }
                        case 2: {
                            drive.Arm.setPosition(0.94);
                            break;
                        }
                        case 3: {
                            drive.Arm.setPosition(0.45);
                            break;
                        }
                    }
//                    drive.telemetry.addData("wobble state", wobbleMode.getValue());
//                    drive.telemetry.addData("desired Arm position", drive.Arm.getDesiredPosition());
//                    drive.telemetry.addData("Arm speed", drive.Arm.getSpeed());
                    if(gamepad.left_trigger.justPressed()){
                        wobbleMode.cycle();
                    }

                    //Hand Toggle
                    if(gamepad.a.justPressed()){
                        handState.toggle();
                    }
                    if(handState.getState()){
                        drive.Hand.setPosition(0.08);
                    }
                    else {
                        drive.Hand.setPosition(0.45);
                    }

                    //Intake Toggle
                    if(gamepad.y.justPressed()){
                        isFeeding.toggle();
                    }
                    if(isFeeding.isPressed()){
                        drive.Intake.setPower(-0.6);
                    }
                    else if(!isReversing.getState()){
                        drive.Intake.setPower(0);
                    }

                    //Intake Reversal
                    if(gamepad.dpad_up.isPressed() && !isFeeding.getState()){
                        isReversing.setState(true);
                        drive.Intake.setPower(0.6);
                    } else {
                        isReversing.setState(false);
                    }


                    //Shooter Toggle
//                    if(gamepad.b.justPressed()){
//                        isShooting.toggle();
//                    }
                    if(gamepad.right_trigger.isPressed()){
                        drive.Shooter.setVelocity(drive.findShooterVelocity(shotDistance, targetHeight));
                    }
                    else drive.Shooter.setVelocity(0);

                    //Shooting Code
                    switch(shoot){
                        case SHOOT: {
                            if(gamepad.x.getState() && gamepad.right_trigger.isPressed()){
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
                    if(gamepad.right_bumper.justPressed()){
                        Trajectory followGoal = drive.trajectoryBuilder(currentPose)
                                .lineToLinearHeading(highGoal)
                                .build();
                        drive.followTrajectoryAsync(followGoal);
                        mode = Mode.PATH_FOLLOWING;
                    }

                    //Auto PowerShot code
                    if(gamepad.left_bumper.justPressed() && !arePowerShooting){
                        arePowerShooting = true;
                        targetHeight = powerShotHeight;
                        Trajectory initialPos = drive.trajectoryBuilder(currentPose)
                                .lineToLinearHeading(powerShot)
                                .build();
                        drive.followTrajectoryAsync(initialPos);
                        mode = Mode.PATH_FOLLOWING;

                    }
                    else if(gamepad.left_bumper.justPressed() && arePowerShooting){
                        Trajectory adjust = drive.trajectoryBuilder(currentPose)
                                .lineToConstantHeading(new Vector2d(currentPose.getX(), currentPose.getY()+6))
                                .build();
                        drive.followTrajectoryAsync(adjust);
                        mode = Mode.PATH_FOLLOWING;
                    }
                    if(gamepad.left_stick_button.isPressed()){
                        arePowerShooting = false;
                        targetHeight = highGoalHeight;
                    }

                    if(gamepad.left_trigger.justPressed()){
                        Trajectory dropWobble = drive.trajectoryBuilder(currentPose)
                                .lineToLinearHeading(wobbleDrop)
                                .build();
                        drive.followTrajectoryAsync(dropWobble);
                        mode = Mode.PATH_FOLLOWING;
                    }

                    //Reset Position
                    if(gamepad.right_stick_button.isPressed()){
                        drive.setPoseEstimate(new Pose2d(63, 14.5, 0));
                    }

                    //Reset Heading
                    if(gamepad.back.justPressed()){
                        drive.resetHeading();
                    }

                    //Field Centric Drive Toggle
                    if(gamepad.start.justPressed()) {
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

