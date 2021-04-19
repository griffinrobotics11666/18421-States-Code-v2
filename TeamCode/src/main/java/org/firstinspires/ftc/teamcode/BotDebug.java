package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Cycle;
import org.firstinspires.ftc.teamcode.util.GamepadEx;

@Config
@TeleOp(name="Bot Debug", group="Discopolus")
public class BotDebug extends LinearOpMode {
    public static double shooterVelocity = 0.0;
    public static double intakePower= 0.0;
    public static double ringPushingPosition = 0.34;
    public static double ringShootingPosition = 0.1;
    public static double armPosition = 0;
    public static double handPosition = 0;
    public static boolean canDrive = true;

    private GamepadEx gamepad;

    private final ElapsedTime shootingClock = new ElapsedTime();
    public static double shootingDelay = 500.0;
    private Cycle turnState = new Cycle(3);
    public static double shootingCooldown = 500.0;
    private enum ShootingState {
        AIM,
        SHOOT,
        RESET,
        WAIT
    }
    private ShootingState shoot = ShootingState.AIM;


    //Auto Aim Testing
    public static double highGoalZ = 91/2.54;
    public static Vector2d highGoal = new Vector2d(70.75, -46.5+12);
    public static double MaxError = 1;

    public static double shootingHeight = 3;
    public static double ShootingAngle = 33;
    public static double ShootingForwardOffset = 18;
    public static double ShooterRadius = 1.5;
    public static double ShooterMultiplier = 2.6;
    public static double LineRequirement = 0;

    public static double MaxStartVelo = 335;
    public static double MinStartVelo = 0;
    private Cycle currShot = new Cycle(3);
    public static PIDCoefficients headingCoefficients = new PIDCoefficients(12,0,0);
    public static PIDFController headingController = new PIDFController(headingCoefficients);
    private PIDFController lateralController = new PIDFController(Bot.TRANSLATIONAL_PID);
    private PIDFController axialController = new PIDFController(Bot.TRANSLATIONAL_PID);

    public static double AngleOffset = 0;
    private double targetAngle;
    private double velocity = 0;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);
        telemetry.addData("","started OpMode");
        telemetry.update();
        gamepad = new GamepadEx(gamepad1);
        Bot bot = new Bot(hardwareMap);
        telemetry.addData("","created Bot instance");
        telemetry.update();
        bot.usingVuforia = false;
        bot.setPoseEstimate(new Pose2d(-63, -25, 0));
        headingController.setInputBounds(-Math.PI, Math.PI);
        bot.initVision();
        waitForStart();
        telemetry.setAutoClear(true);
        while (opModeIsActive()) {
            if(turnState.getValue()==1){
                bot.Shooter.setVelocity(shooterVelocity);
                bot.Trigger.setPosition(ringPushingPosition);
            }
            bot.Intake.setPower(intakePower);
            bot.Arm.setPosition(armPosition);
            bot.Hand.setPosition(handPosition);
            Pose2d currentPose = bot.getPoseEstimate();
            Pose2d currentVelocity = bot.getPoseVelocity();
            telemetry.addData("Shooter Velocity",bot.Shooter.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("computed Shooter Velocity", (bot.Shooter.getVelocity()/28)*2*Math.PI);
            telemetry.addData("Shooter rpm",(bot.Shooter.getVelocity()/28)*60);
            telemetry.addData("Shooter ticks/s",bot.Shooter.getVelocity());
            telemetry.addData("Desired rpm", ((velocity/ShooterRadius)/(2*Math.PI))*60);
            telemetry.addData("Desired ticks/s", ((velocity/ShooterRadius)/(2*Math.PI))*28*ShooterMultiplier);
            telemetry.addData("GamepadEx x",gamepad.x.getState());
            telemetry.addData("GamepadEx x last: ", gamepad.x.getLastState());
            telemetry.addData("turnState: ", turnState.getValue());
            telemetry.addData("velocity for shoot: ", velocity);
            telemetry.addData("Target angle", targetAngle);
//Shooting Code
            switch(shoot){
                case AIM: {
                    if(gamepad.x.justPressed() && turnState.getValue() == 1 && currentPose.getX()<=LineRequirement){
                        turnState.cycle();
                        targetAngle = Math.atan2(highGoal.minus(currentPose.vec()).getY(),highGoal.minus(currentPose.vec()).getX())-currentPose.getHeading()-AngleOffset;
                        headingController.reset();
                        headingController.setTargetPosition(targetAngle);
                    }
                    if(turnState.getValue()==2){
                        double headingInput = (headingController.update(currentPose.getHeading()) * BotConstants.kV) * BotConstants.TRACK_WIDTH;
//                        headingController.update(currentPose.getHeading());
                        telemetry.addData("error distance", headingController.getLastError()-Math.toRadians(5));
                        if(Math.abs(headingController.getLastError())>Math.toRadians(5)){
                            bot.setDriveSignal(new DriveSignal(new Pose2d(new Vector2d(), headingInput)));
                        }
                        else {
                            bot.setWeightedDrivePower(new Pose2d());
                            turnState.cycle();
                            break;
                        }
                    }
                    if(turnState.getValue()==3){
                        double maxVelo = MaxStartVelo;
                        double minVelo = MinStartVelo;
                        double velo = 0;
                        double x = Math.cos(Math.toRadians(ShootingAngle));
                        double y = Math.sin(Math.toRadians(ShootingAngle));
                        double i = highGoalZ;
                        while(Math.abs(i)>MaxError){
                            velo = (maxVelo+minVelo)/2;
                            double t = highGoal.distTo(currentPose.vec())/(velo*x);
                            i = highGoalZ-(((-385.827/2)*t*t)+(velo*y*t)+shootingHeight);
                            if(Math.signum(i)==1){
                                minVelo = velo;
                            }
                            if(Math.signum(i)==-1){
                                maxVelo = velo;
                            }
                            if(Math.signum(i)==0){
                                break;
                            }
                        }
                        velocity = velo;
                        telemetry.addData("found velocity: ", velo/ShooterRadius);
                        if(bot.Shooter.getVelocity()>=((velo/ShooterRadius)/(2*Math.PI))*28*ShooterMultiplier){
                            shoot = ShootingState.SHOOT;
                            currShot.setValue(1);
                            break;
                        }
                        bot.Shooter.setVelocity(((velo/ShooterRadius)/(2*Math.PI))*28*ShooterMultiplier);
                    }
                    break;
                }
                case SHOOT: {
                    shootingClock.reset();
                    bot.Trigger.setPosition(ringShootingPosition);
                    shoot = ShootingState.RESET;
                    break;
                }
                case RESET: {
                    if(shootingClock.milliseconds()>= shootingDelay){
                        shootingClock.reset();
                        bot.Trigger.setPosition(ringPushingPosition);
                        shoot = ShootingState.WAIT;
                        break;
                    }
                }
                case WAIT: {
                    if(shootingClock.milliseconds()>= shootingCooldown){
                        shootingClock.reset();
                        if(currShot.getValue()==currShot.numValues()){
                            shoot= ShootingState.AIM;
                            turnState.cycle();
                        }
                        else {
                            shoot = ShootingState.SHOOT;
                        }
                        currShot.cycle();
                        break;
                    }
                }
            }

            if(canDrive && turnState.getValue()==1){
//                Pose2d targetVel = new Pose2d(
//                        new Vector2d(
//                                -gamepad1.left_stick_y,
//                                -gamepad1.left_stick_x
//                        ).div(new Vector2d(
//                                -gamepad1.left_stick_y,
//                                -gamepad1.left_stick_x
//                        ).norm()).times(BotConstants.MAX_VEL),
//                        -gamepad1.right_stick_x*BotConstants.MAX_ANG_VEL
//                );
//                Pose2d targetRobotVel = Kinematics.fieldToRobotVelocity(currentPose,targetVel);
//
//                axialController.setTargetPosition(targetRobotVel.getX());
//                lateralController.setTargetPosition(targetRobotVel.getY());
//                headingController.setTargetPosition(targetRobotVel.getHeading());
//
//                double axialCorrection = axialController.update(currentVelocity.getX());
//                double lateralCorrection = lateralController.update(currentVelocity.getX());
//                double headingCorrection = headingController.update(currentVelocity.getHeading());
//
//                Pose2d correctedVelocity = targetRobotVel.plus(new Pose2d(
//                        axialCorrection,
//                        lateralCorrection,
//                        headingCorrection
//                ));
//
//                bot.setDriveSignal(new DriveSignal(correctedVelocity));
                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of that heading
                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ).rotated(-bot.getPoseEstimate().getHeading());

                // Pass in the rotated input + right stick value for rotation
                // Rotation is not part of the rotated input thus must be passed in separately
                bot.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x
                        )
                );
            }
            gamepad.update();
            bot.update();
        }

        bot.deactivateVision();
    }

}
