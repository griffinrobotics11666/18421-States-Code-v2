package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.GVFFollower;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.PathFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.UltimateGoalLocalizer;
import org.firstinspires.ftc.teamcode.util.UltimateGoalTfod;
import org.firstinspires.ftc.teamcode.util.VuforiaUtil;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.BotConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.BotConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.BotConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.BotConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.BotConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.BotConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.BotConstants.kA;
import static org.firstinspires.ftc.teamcode.BotConstants.kStatic;
import static org.firstinspires.ftc.teamcode.BotConstants.kV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;

/*

  _____   _______   ___ ___   _______   _____
 | _   | |   _   | |   Y   | |       | | _   |
 |.|   | |.  |   | |   |   | |___|   | |.|   |
 `-|.  | |.  _   | |____   |  /  ___/  `-|.  |
   |:  | |:  1   |     |:  | |:  1  \    |:  |
   |::.| |::.. . |     |::.| |::.. . |   |::.|
   `---' `-------'     `---' `-------'   `---'

 ______  _____ _______ _______  _____   _____   _____         _     _ _______
 |     \   |   |______ |       |     | |_____] |     | |      |     | |______
 |_____/ __|__ ______| |_____  |_____| |       |_____| |_____ |_____| ______|

 */

/**
 * Team 18421's robot for the 2020-2021 Ultimate Goal season.
 */
public class Bot extends MecanumDrive {
    //PID Coefficients
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(1.1, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(4, 0, 0);
    public static double LATERAL_MULTIPLIER = 1.0719683964;
    public static PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(40,0,3,15); //Not necessary
    public static PIDFCoefficients INTAKE_PID = new PIDFCoefficients(0, 0, 0,0); //Even less necessary

    //Drive Weights
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    //Vision Goodies
    private static final String VUFORIA_KEY = "AdYYXLj/////AAABmbrz6/MNLUKlnU5JIPwkiDQ5jX+GIjfuIEgba3irGu46iS/W1Q9Z55uLSl31zGtBX3k5prkoSK6UxLR9gyvyIwSzRe2FOFGHEvJ19uG+pqiJJfkaRb0mCUkrx4U/fH6+Agp+7lOHB8IYjziNSuBMgABbrii5tAQiXOGfGojY+IQ/enBoy+zWiwVBx9cPRBsEHu+ipK6RXQe7CeODCRN8anBfAsn5b2BoO9lcGE0DgZdRysyByQ4wuwNQxKjba18fnzSDWpm12Brx3Ao1vkGYxTyLQfsON5VotphvWwoZpoyD+Iav/yQmOxrQDBLox6SosF8jqG9sUC5LAAdiRIWr6sNRrGzeCtsHSJBplHboPMB3";
    private VuforiaLocalizer vuforia;
    private UltimateGoalLocalizer vuforiaLocalizer;
    public static boolean usingVuforia = true;
    private UltimateGoalTfod tfod;
    private OpenCvCamera camera;

    //Finite State Machine Stuffs
    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY,
        FOLLOW_PATH
    }
    private Mode mode;

    //Dashboard functionality
    private FtcDashboard dashboard;
    public MultipleTelemetry telemetry;

    //Turning Stuff
    private NanoClock clock;
    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;
    private Pose2d lastPoseOnTurn;
    public static double MaxJerk = 70;

    //Trajectory Following Bits
    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private TrajectoryFollower follower;

    //Path Following Bobs
    private PathFollower pathFollower;

    //Pose History Variables (For displaying trajectories on dashboard)
    public static int POSE_HISTORY_LIMIT = 100;
    private LinkedList<Pose2d> poseHistory;

    /*Hardware Variables*/
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;
    public DcMotorEx Shooter;
    public DcMotorEx Intake;
    public Servo Trigger;
    public Servo linearSlide;
    public Servo Arm;
    public Servo Hand;
    public Servo Latch;
    public WebcamName Webcam;
    public VoltageSensor batteryVoltageSensor;

    //Experimental Ring Capacity Detection
    private double feedingPeak;
    private double shootingPeak;
    private boolean isFeeding = false;
    private boolean isShooting = false;
    public int numRings = 0;

    public Bot(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = new MultipleTelemetry(dashboard.getTelemetry());
        
        //This variable is initialized so that that error NEVER EVER happens again
        mode = Mode.IDLE;

        clock = NanoClock.system();

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, DriveConstants.TRACK_WIDTH)
        ));
        accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        pathFollower = new GVFFollower(MAX_VEL, MAX_ACCEL, new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 2, 1);

        poseHistory = new LinkedList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(IMUparameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_back");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_back");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        Shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        Intake = hardwareMap.get(DcMotorEx.class, "feeder");
        Trigger = hardwareMap.get(Servo.class, "trigger");
        Arm = hardwareMap.get(Servo.class, "arm");
        Hand = hardwareMap.get(Servo.class, "hand");
        Latch = hardwareMap.get(Servo.class, "latch");
        Webcam = hardwareMap.get(WebcamName.class, "webcam");

        MotorConfigurationType ShooterType = Shooter.getMotorType().clone();
        ShooterType.setAchieveableMaxRPMFraction(1.0);
        Shooter.setMotorType(ShooterType);

        MotorConfigurationType IntakeType = Intake.getMotorType().clone();
        IntakeType.setAchieveableMaxRPMFraction(1.0);
        Intake.setMotorType(IntakeType);

        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

//        Shooter.setVelocityPIDFCoefficients(
//                SHOOTER_PID.p, SHOOTER_PID.i, SHOOTER_PID.d, SHOOTER_PID.f * 12 / batteryVoltageSensor.getVoltage()
//        );
        
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new BotOdometry(hardwareMap));
        
        //Vuforia Object Initialization
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
             * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
             */
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    
            // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = Webcam;
            parameters.useExtendedTracking = false;
    
            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Vuforia and Tensorflow Initialization
            VuforiaUtil.CameraState cameraState = new VuforiaUtil.CameraState(VuforiaLocalizer.CameraDirection.BACK, VuforiaUtil.CameraDirection.LANDSCAPE, VuforiaUtil.CameraDirection.FORWARD, 0, 0, 0, 0);
            vuforiaLocalizer = new UltimateGoalLocalizer(vuforia, cameraState);
            tfod = new UltimateGoalTfod(vuforia, hardwareMap);

        //OpenCV Initialization
//        camera = OpenCvCameraFactory.getInstance().createWebcam(Webcam, cameraMonitorViewId);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startTangent) {
        return new TrajectoryBuilder(startPose, startTangent, velConstraint, accelConstraint);
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public PathBuilder pathBuilder(Pose2d startPose){
        return new PathBuilder(startPose);
    }

    public PathBuilder pathBuilder(Pose2d startPose, boolean reversed){
        return new PathBuilder(startPose, reversed);
    }

    public PathBuilder pathBuilder(Pose2d startPose, double startTangent){
        return new PathBuilder(startPose, startTangent);
    }

    public void followPathAsync(Path path) {
        pathFollower.followPath(path);
        mode = Mode.FOLLOW_PATH;
    }
//    public void followPathAsync(Path path, Obstacle[] obstacles){
//        pathFollower.followPath(path, obstacles);
//        mode = Mode.FOLLOW_PATH;
//    }

    public void followPath(Path path) {
        followPathAsync(path);
        waitForIdle();
    }
//    public void followPath(Path path, Obstacle[] obstacles) {
//        followPathAsync(path, obstacles);
//        waitForIdle();
//    }

    public void forceIdle() {
        mode = Mode.IDLE;
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                DriveConstants.MAX_ANG_VEL,
                MAX_ANG_ACCEL,
                Math.toRadians(MaxJerk)
        );

        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }
    
    public void initVision(){
//        tfod.activate();
        vuforiaLocalizer.activate();
//        camera.openCameraDeviceAsync(() -> {
//            camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            camera.setPipeline(new RingPipeline());
//            dashboard.startCameraStream(camera, 0);
//        });
    }
    
    public void deactivateVision(){
        tfod.deactivate();
        vuforiaLocalizer.deactivate();
    }

    public String detectStarterStack(int iterations, int milliseconds) {
        tfod.activate();
        for(int i = 0; i<iterations; i++){
            tfod.update();
            try {
                Thread.sleep(milliseconds);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        tfod.deactivate();
        return tfod.visibleTarget();
    }

    public String detectStarterStack(int milliseconds) {
        ElapsedTime time = new ElapsedTime();
        tfod.activate();
        time.reset();
        while(time.milliseconds() < milliseconds) {
            tfod.update();
        }
        tfod.deactivate();
        return tfod.visibleTarget();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();
        Pose2d currentVelocity = getPoseVelocity();
        PoseStorage.currentPose = currentPose;

        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        telemetry.addData("mode", mode);

        telemetry.addData("x", currentPose.getX());
        telemetry.addData("y", currentPose.getY());
        telemetry.addData("heading", currentPose.getHeading());
        telemetry.addData("velocity", currentVelocity);

        telemetry.addData("xError", lastError.getX());
        telemetry.addData("yError", lastError.getY());
        telemetry.addData("headingError", lastError.getHeading());

//        Vuforia Localization
        if(vuforiaLocalizer.targetVisible){
            /*There's an option to disable automatic vuforia localization (usingVuforia)
              And a velocity cap on it so that vuforia doesn't try to read blurry images
             */
            fieldOverlay.setStroke("#eb3434");
            DashboardUtil.drawRobot(fieldOverlay, vuforiaLocalizer.lastPose);
            telemetry.addData("supposed vuforia pose",vuforiaLocalizer.lastPose);
            assert currentVelocity != null;
            if(currentVelocity.getX()+currentVelocity.getY()+currentVelocity.getHeading()<0.5 && usingVuforia) {
                setPoseEstimate(vuforiaLocalizer.lastPose);
            }
        }
        vuforiaLocalizer.update();

        //OpenCV debugging
//        camera.setPipeline(new RingPipeline());

        //Experimental Ring Capacity Detection
        telemetry.addData("Intake current", Intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Shooter current", Shooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Num rings", numRings);
//        if(Intake.getCurrent(CurrentUnit.AMPS)<2.6){
//            if(feedingPeak>3.0){
//                numRings++;
//            }
//            feedingPeak = 0.0;
//        }
//        else {
//            if(Intake.getCurrent(CurrentUnit.AMPS)>feedingPeak){
//                feedingPeak = Intake.getCurrent(CurrentUnit.AMPS);
//            }
//        }
//
//        if(Shooter.getCurrent(CurrentUnit.AMPS)<2.0){
//            if(shootingPeak>2.0 && shootingPeak<6.0){
//                numRings--;
//            }
//            shootingPeak = 0.0;
//        }
//        else {
//            if(Shooter.getCurrent(CurrentUnit.AMPS)>shootingPeak){
//                shootingPeak = Shooter.getCurrent(CurrentUnit.AMPS);
//            }
//        }

        //Finite State Machine stuffs
        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

//                fieldOverlay.setStroke("#4CAF50");
//                DashboardUtil.drawRobot(fieldOverlay, newPose);

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_PATH: {
                setDriveSignal(pathFollower.update(currentPose));

                Path path = pathFollower.getPath();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, path);
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, path.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

                if (!pathFollower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }
        
    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(DriveConstants.encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(DriveConstants.encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
    public void resetHeading(){
        setExternalHeading(0.0);
    }
}
