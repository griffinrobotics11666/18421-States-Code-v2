package org.firstinspires.ftc.teamcode.util.roadrunnerEx

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.util.GuidingVectorField
import kotlin.math.sign

/**
 * Guiding vector field for effective path following described in section III, eq. (9) of
 * [1610.04391.pdf](https://arxiv.org/pdf/1610.04391.pdf). Implementation note: 2D parametric curves are used to
 * describe paths instead of implicit curves of the form f(x,y) = 0 as described in the paper (which dramatically
 * affects the cross track error calculation). Includes circular obstacle avoidance.
 *
 * @param path path to follow (interpolator is ignored)
 * @param kN path normal weight (see eq. (9))
 * @param errorMapFunc custom error mapping (see eq. (4))
 * @param obstacles list of obstacles to avoid (see [Obstacle])
 */
class GuidingVectorFieldEx(
        private val path: Path,
        private val kN: Double,
        private val errorMapFunc: (Double) -> Double = { it },
        private val obstacles: Array<Obstacle>?
) {
    constructor(path: Path, kN: Double, errorMapFunc: (Double) -> Double):
            this(path, kN, errorMapFunc, null)
    /**
     * Container for the direction of the GVF and intermediate values used in its computation.
     *
     * @param vector normalized direction vector of the GVF
     * @param pathPoint point on the path from the projection
     * @param displacement displacement along the path of [pathPoint]
     * @param error signed cross track error (distance between [pathPoint] and the query point)
     */
    data class GVFResultEx(
            val vector: Vector2d,
            val pathPoint: Pose2d,
            val displacement: Double,
            val error: Double
    )

    /**
     * Returns the normalized value of the vector field at the given point along with useful intermediate computations.
     */
    fun getExtended(x: Double, y: Double, projectGuess: Double = path.length() / 2.0): GVFResultEx {
        val point = Vector2d(x, y)
        val displacement = path.fastProject(Vector2d(x, y), projectGuess)
        val pathPoint = path[displacement]
        val tangent = path.deriv(displacement).vec()
        val pathToPoint = point - pathPoint.vec()
        val orientation = -sign(pathToPoint.x * tangent.y - pathToPoint.y * tangent.x)
        val error = orientation * pathToPoint.norm()
        val normal = tangent.rotated(Math.PI / 2.0)
        var avoidance = Vector2d(0.0,0.0)
        if (obstacles != null) {
            for(obstacle in obstacles){
                avoidance = avoidance.plus(obstacle.get(x,y))
            }
        }
        val vector = tangent - (normal * kN * errorMapFunc(error)) + avoidance
        return GVFResultEx(
                vector / vector.norm(),
                pathPoint,
                displacement,
                error
        )
    }

    /**
     * Returns the normalized value of the vector field at the given point.
     */
    operator fun get(x: Double, y: Double) = getExtended(x, y).vector
}