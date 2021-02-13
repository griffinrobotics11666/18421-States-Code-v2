package org.firstinspires.ftc.teamcode.util.roadrunnerEx

import com.acmerobotics.roadrunner.geometry.Vector2d
import kotlin.math.tanh
import kotlin.math.PI

/**
 * Obstacle class for circulating vector field obstacle avoidance. To be used with [GuidingVectorFieldEx]
 * See [this paper](https://etd.ohiolink.edu/apexprod/rws_etd/send_file/send?accession=ohiou1530874969780028&disposition=inline) for more information.
 *
 * @param Point the location of the obstacle to avoid
 * @param radius the area of effect around the obstacle (how far away the robot will stay)
 * @param G coefficient for convergence vector (see section 3.2.2)
 * @param H coefficient for circulation vector (see eq. 3.14)
 * @param k coefficient for decay radius (see eq. 3.25)
 * @param multiplier shorthand for declaring both G and H (makes them equal)
 */
class Obstacle(
        private val Point: Vector2d,
        private val radius: Double,
        private val G: Double,
        private val H: Double,
        private val k: Double
        ) {
    constructor(Point: Vector2d, radius: Double, multiplier: Double):
        this(Point, radius, multiplier, multiplier, 2.8)
    constructor(Point: Vector2d, radius: Double, multiplier: Double, k: Double):
            this(Point, radius, multiplier, multiplier, k)
    constructor(Point: Vector2d, radius: Double):
            this(Point, radius, -1.0, 2.1, 2.8)

    /**
     * Returns the normalized value of the vector field at the given point.
     */
    fun get(x: Double, y: Double): Vector2d{
        val point = Vector2d(x,y)
        val error = Point - point
        val conv = error*2.0
        val circ = Vector2d(2*error.y, -2*error.x)
        val vector = (conv/conv.norm())*G + (circ/circ.norm())*H
        val P = -tanh((2*PI*Point.distTo(point))/(radius*k)-PI)+1
        return (vector/vector.norm())*P
    }
}