package org.firstinspires.ftc.teamcode.util.roadrunnerEx

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.followers.PathFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.NanoClock
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * Even-More-State-of-the-art path follower based on the [GuidingVectorFieldEx] with obstacle avoidance capabilities.
 *
 * @param maxVel maximum velocity
 * @param maxAccel maximum acceleration
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param kN normal vector weight (see [GuidingVectorFieldEx])
 * @param kOmega proportional heading gain
 * @param errorMapFunc error map function (see [GuidingVectorFieldEx])
 * @param clock clock
 */
class GVFFollowerEx @JvmOverloads constructor(
        private val maxVel: Double,
        private val maxAccel: Double,
        admissibleError: Pose2d,
        private val kN: Double,
        private val kOmega: Double,
        private val errorMapFunc: (Double) -> Double = { it },
        clock: NanoClock = NanoClock.system()
) : PathFollower(admissibleError, clock) {
    private lateinit var gvf: GuidingVectorFieldEx
    private var lastUpdateTimestamp: Double = 0.0
    private var lastVel: Double = 0.0
    private var lastProjDisplacement: Double = 0.0

    override var lastError: Pose2d = Pose2d()

    override fun followPath(path: Path) {
        gvf = GuidingVectorFieldEx(path, kN, errorMapFunc)
        lastUpdateTimestamp = clock.seconds()
        lastVel = 0.0
        lastProjDisplacement = 0.0
        super.followPath(path)
    }
    /**
     * @param obstacles list of obstacles to avoid (see [Obstacle])
     */
    fun followPath(path: Path, obstacles: Array<Obstacle>) {
        gvf = GuidingVectorFieldEx(path, kN, errorMapFunc, obstacles)
        lastUpdateTimestamp = clock.seconds()
        lastVel = 0.0
        lastProjDisplacement = 0.0
        super.followPath(path)
    }

    override fun internalUpdate(currentPose: Pose2d): DriveSignal {
        val gvfResult = gvf.getExtended(currentPose.x, currentPose.y, lastProjDisplacement)

        val desiredHeading = atan2(gvfResult.vector.y, gvfResult.vector.x)
        val headingError = Angle.normDelta(desiredHeading - currentPose.heading)

        // TODO: implement this or nah? ref eqs. (18), (23), and (24)
        val desiredOmega = 0.0
        val omega = desiredOmega + kOmega * headingError

        // basic online motion profiling
        val timestamp = clock.seconds()
        val dt = timestamp - lastUpdateTimestamp
        val remainingDistance = currentPose.vec() distTo path.end().vec()
        val maxVelToStop = sqrt(2 * maxAccel * remainingDistance)
        val maxVelFromLast = lastVel + maxAccel * dt
        val velocity = minOf(maxVelFromLast, maxVelToStop, maxVel)

        lastUpdateTimestamp = timestamp
        lastVel = velocity
        lastProjDisplacement = gvfResult.displacement

        val targetPose = path[gvfResult.displacement]

        lastError = Kinematics.calculatePoseError(targetPose, currentPose)

        // TODO: GVF acceleration FF?
        return DriveSignal(Pose2d(velocity, 0.0, omega))
    }
}