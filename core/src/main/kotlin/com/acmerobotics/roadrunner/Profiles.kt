@file:JvmName("Profiles")

package com.acmerobotics.roadrunner

import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt
import kotlin.math.withSign

/**
 * Time parameter for [DualNum].
 */
class Time

/**
 * Motion profile parameterized by displacement.
 */
data class DisplacementProfile(
    @JvmField
    val disps: List<Double>,
    @JvmField
    val vels: List<Double>,
    @JvmField
    val accels: List<Double>,
) {
    @JvmField
    val length = disps.last()

    init {
        require(disps.size == vels.size)
        require(disps.size == accels.size + 1)
    }

    operator fun get(x: Double): DualNum<Time> {
        val index = disps.binarySearch(x)
        return when {
            index >= disps.lastIndex -> DualNum(doubleArrayOf(x, vels[index], 0.0))
            index >= 0 -> DualNum(doubleArrayOf(x, vels[index], accels[index]))
            else -> {
                val insIndex = -(index + 1)
                when {
                    insIndex <= 0 -> DualNum(doubleArrayOf(x, vels.first(), 0.0))
                    insIndex >= disps.size -> DualNum(doubleArrayOf(x, vels.last(), 0.0))
                    else -> {
                        val dx = x - disps[insIndex - 1]
                        val v0 = vels[insIndex - 1]
                        val a = accels[insIndex - 1]

                        DualNum(
                            doubleArrayOf(
                                x,
                                sqrt(v0 * v0 + 2 * a * dx),
                                a
                            )
                        )
                    }
                }
            }
        }
    }
}

class CancelableProfile(
    @JvmField val baseProfile: DisplacementProfile,
    @JvmField val disps: List<Double>,
    @JvmField val minAccels: List<Double>
) {
    fun cancel(x: Double): DisplacementProfile {
        val newDisps = mutableListOf(0.0)
        val vels = mutableListOf(baseProfile[x][1])
        val accels = mutableListOf<Double>()

        val rawIndex = disps.binarySearch(x)
        val beginIndex = if (rawIndex >= 0) {
            rawIndex + 1
        } else {
            val insIndex = -(rawIndex + 1)
            if (insIndex == 0) {
                newDisps.add(-x)
                vels.add(vels.last())
                accels.add(0.0)

                1
            } else {
                insIndex
            }
        }

        val targetVel = baseProfile.vels.last()
        for (index in beginIndex..disps.lastIndex) {
            val v = vels.last()
            val a = minAccels[index - 1]

            val targetDisp = newDisps.last() + (targetVel * targetVel - v * v) / (2 * a)
            if (x + targetDisp > disps[index]) {
                newDisps.add(disps[index] - x)
                vels.add(sqrt(v * v + 2 * a * (disps[index] - disps[index - 1])))
                accels.add(a)
            } else {
                newDisps.add(targetDisp)
                vels.add(targetVel)
                accels.add(a)

                break
            }
        }

        return DisplacementProfile(newDisps, vels, accels)
    }
}

private fun timeScan(p: DisplacementProfile): List<Double> {
    val times = mutableListOf(0.0)
    for (i in p.accels.indices) {
        times.add(
            times.last() +
                if (p.accels[i] == 0.0) {
                    (p.disps[i + 1] - p.disps[i]) / p.vels[i]
                } else {
                    (p.vels[i + 1] - p.vels[i]) / p.accels[i]
                }
        )
    }
    return times
}

/**
 * Motion profile parameterized by time.
 */
// guaranteed monotonic
data class TimeProfile @JvmOverloads constructor(
    @JvmField
    val dispProfile: DisplacementProfile,
    @JvmField
    val times: List<Double> = timeScan(dispProfile),
) {
    @JvmField
    val duration = times.last()

    init {
        require(times.size == dispProfile.disps.size)
    }

    operator fun get(t: Double): DualNum<Time> {
        val index = times.binarySearch(t)
        return when {
            index >= times.lastIndex ->
                DualNum(
                    doubleArrayOf(
                        dispProfile.disps[index], dispProfile.vels[index], 0.0
                    )
                )
            index >= 0 ->
                DualNum(
                    doubleArrayOf(
                        dispProfile.disps[index], dispProfile.vels[index], dispProfile.accels[index]
                    )
                )
            else -> {
                val insIndex = -(index + 1)
                when {
                    insIndex <= 0 -> {
                        val v = dispProfile.vels.first()
                        DualNum(doubleArrayOf(v * t, v, 0.0))
                    }
                    insIndex >= times.size -> {
                        val v = dispProfile.vels.last()
                        DualNum(doubleArrayOf(dispProfile.length + v * (t - duration), v, 0.0))
                    }
                    else -> {
                        val dt = t - times[insIndex - 1]
                        val x0 = dispProfile.disps[insIndex - 1]
                        val v0 = dispProfile.vels[insIndex - 1]
                        val a = dispProfile.accels[insIndex - 1]

                        DualNum(
                            doubleArrayOf(
                                (0.5 * a * dt + v0) * dt + x0,
                                a * dt + v0,
                                a
                            )
                        )
                    }
                }
            }
        }
    }

    fun inverse(x: Double): Double {
        val index = dispProfile.disps.binarySearch(x)
        return when {
            index >= dispProfile.disps.lastIndex -> times[index]
            index >= 0 -> times[index]
            else -> {
                val insIndex = -(index + 1)
                when {
                    insIndex <= 0 -> 0.0
                    insIndex >= times.size -> duration
                    else -> {
                        val dx = x - dispProfile.disps[insIndex - 1]
                        val t0 = times[insIndex - 1]
                        val v0 = dispProfile.vels[insIndex - 1]
                        val a = dispProfile.accels[insIndex - 1]

                        if (a == 0.0) {
                            t0 + dx / v0
                        } else {
                            t0 + sqrt(((v0 * v0 / a) + 2 * dx) / a).withSign(a) - v0 / a
                        }
                    }
                }
            }
        }
    }
}

/**
 * Computes an exact, time-optimal profile.
 *
 * @param[beginEndVel] beginning and ending velocity (must be the same to guarantee feasibility)
 * @param[maxVel] positive
 * @param[minAccel] negative
 * @param[maxAccel] positive
 */
fun constantProfile(
    length: Double,
    beginEndVel: Double,
    maxVel: Double,
    minAccel: Double,
    maxAccel: Double,
) = profile(length, beginEndVel, { maxVel }, { minAccel }, { maxAccel }, length)

/**
 * Computes an approximately time-optimal profile by sampling the constraints according to the resolution [resolution].
 *
 * @param[beginEndVel] beginning and ending velocity, non-negative (must be the same to guarantee feasibility)
 * @param[maxVel] always returns positive
 * @param[minAccel] always returns negative
 * @param[maxAccel] always returns positive
 */
fun profile(
    length: Double,
    beginEndVel: Double,
    maxVel: (Double) -> Double,
    minAccel: (Double) -> Double,
    maxAccel: (Double) -> Double,
    resolution: Double,
): CancelableProfile {
    require(length > 0.0)
    require(resolution > 0.0)
    require(beginEndVel >= 0.0)

    val samples = max(1, ceil(length / resolution).toInt())

    val disps = rangeMiddle(0.0, length, samples)
    val maxVels = disps.map(maxVel)
    val minAccels = disps.map(minAccel)
    val maxAccels = disps.map(maxAccel)

    return profile(
        range(0.0, length, samples + 1),
        beginEndVel, maxVels, minAccels, maxAccels
    )
}

/**
 * Computes an approximately time-optimal profile from center-sampled constraints.
 *
 * @param[beginEndVel] beginning and ending velocity (must be the same to guarantee feasibility)
 * @param[maxVels] all positive
 * @param[minAccels] all negative
 * @param[maxAccels] all positive
 */
fun profile(
    disps: List<Double>,
    beginEndVel: Double,
    maxVels: List<Double>,
    minAccels: List<Double>,
    maxAccels: List<Double>,
): CancelableProfile {
    require(maxVels.size == minAccels.size)
    require(maxVels.size == maxAccels.size)

    return CancelableProfile(
        merge(
            forwardProfile(disps, beginEndVel, maxVels, maxAccels),
            backwardProfile(disps, maxVels, beginEndVel, minAccels),
        ),
        disps, minAccels
    )
}

/**
 * Computes an approximately time-optimal forward profile by sampling the constraints according to the resolution
 * [resolution]. No restriction is imposed on the minimum acceleration.
 *
 * @param[beginVel] beginning velocity, non-negative
 * @param[maxVel] always returns positive
 * @param[maxAccel] always returns positive
 */
fun forwardProfile(
    length: Double,
    beginVel: Double,
    maxVel: (Double) -> Double,
    maxAccel: (Double) -> Double,
    resolution: Double,
): DisplacementProfile {
    val samples = max(1, ceil(length / resolution).toInt())

    val disps = rangeMiddle(0.0, length, samples)
    val maxVels = disps.map(maxVel)
    val maxAccels = disps.map(maxAccel)
    return forwardProfile(
        range(0.0, length, samples + 1),
        beginVel, maxVels, maxAccels
    )
}

/**
 * Computes an approximately time-optimal forward profile from the center-sampled constraints. No restriction is imposed
 * on the minimum acceleration.
 *
 * The procedure uses a variant of the approach described in [section 14.6.3.5](http://lavalle.pl/planning/node794.html)
 * of LaValle's excellent book on planning.
 *
 * @param[disps] displacement interval endpoints
 * @param[beginVel] beginning velocity, non-negative
 * @param[maxVels] all positive
 * @param[maxAccels] all positive
 */
@Suppress("NAME_SHADOWING")
fun forwardProfile(
    disps: List<Double>,
    beginVel: Double,
    maxVels: List<Double>,
    maxAccels: List<Double>,
): DisplacementProfile {
    val newDisps = mutableListOf(0.0)
    val vels = mutableListOf(beginVel)
    val accels = mutableListOf<Double>()

    maxVels
        .zip(maxAccels)
        .zip(disps.drop(1))
        .fold(disps[0]) { beginDisp, (c, endDisp) ->
            val (maxVel, maxAccel) = c

            val beginVel = vels.last()
            if (beginVel >= maxVel) {
                newDisps.add(endDisp)
                vels.add(maxVel)
                accels.add(0.0)
            } else {
                val endVel = sqrt(beginVel * beginVel + 2 * maxAccel * (endDisp - beginDisp))
                if (endVel <= maxVel) {
                    newDisps.add(endDisp)
                    vels.add(endVel)
                    accels.add(maxAccel)
                } else {
                    val accelDx = (maxVel * maxVel - beginVel * beginVel) / (2 * maxAccel)

                    newDisps.add(beginDisp + accelDx)
                    vels.add(maxVel)
                    accels.add(maxAccel)

                    newDisps.add(endDisp)
                    vels.add(maxVel)
                    accels.add(0.0)
                }
            }

            endDisp
        }

    return DisplacementProfile(newDisps, vels, accels)
}

/**
 * Computes an approximately time-optimal backward profile by sampling the constraints according to the resolution
 * [resolution]. No restriction is imposed on the minimum acceleration.
 *
 * @param[maxVel] always returns positive
 * @param[endVel] ending velocity, non-negative
 * @param[minAccel] always returns negative
 */
fun backwardProfile(
    length: Double,
    maxVel: (Double) -> Double,
    endVel: Double,
    minAccel: (Double) -> Double,
    resolution: Double,
): DisplacementProfile {
    require(endVel >= 0.0)

    val samples = max(1, ceil(length / resolution).toInt())

    val disps = rangeMiddle(0.0, length, samples)
    // TODO: verify signs?
    val maxVels = disps.map(maxVel)
    val minAccels = disps.map(minAccel)
    return backwardProfile(
        range(0.0, length, samples + 1),
        maxVels, endVel, minAccels
    )
}

/**
 * Computes an approximately time-optimal backward profile from the center-sampled constraints. No restriction is imposed
 * on the maximum acceleration.
 *
 * @param[disps] displacement interval endpoints
 * @param[maxVels] all positive
 * @param[endVel] ending velocity, non-negative
 * @param[minAccels] all negative
 */
fun backwardProfile(
    disps: List<Double>,
    maxVels: List<Double>,
    endVel: Double,
    minAccels: List<Double>,
) = forwardProfile(
    disps, endVel, maxVels.reversed(), minAccels.reversed().map { -it }
).let {
    DisplacementProfile(
        it.disps.map { x -> it.length - x }.reversed(),
        it.vels.reversed(),
        it.accels.reversed().map { a -> -a },
    )
}

/**
 * Merges [p1] and [p2] into another profile with the minimum velocity of the two at every point.
 */
fun merge(p1: DisplacementProfile, p2: DisplacementProfile): DisplacementProfile {
    val disps = mutableListOf(0.0)
    val vels = mutableListOf(min(p1.vels[0], p2.vels[0]))
    val accels = mutableListOf<Double>()

    var lastMin1 = p1.vels[0] < p2.vels[0]

    var i = 1
    var j = 1
    while (i < p1.disps.size && j < p2.disps.size) {
        val endDisp = min(p1.disps[i], p2.disps[j])
        val accel1 = p1.accels[i - 1]
        val accel2 = p2.accels[j - 1]

        val (endVel1, endVel2) =
            if (p1.disps[i] == p2.disps[j]) {
                i++
                j++
                Pair(
                    p1.vels[i - 1],
                    p2.vels[j - 1],
                )
            } else if (p1.disps[i] < p2.disps[j]) {
                i++
                Pair(
                    p1.vels[i - 1],
                    sqrt(
                        p2.vels[j] * p2.vels[j] +
                            2 * accel2 * (p1.disps[i - 1] - endDisp)
                    )
                )
            } else {
                j++
                Pair(
                    sqrt(
                        p1.vels[i] * p1.vels[i] +
                            2 * accel1 * (p2.disps[j - 1] - endDisp)
                    ),
                    p2.vels[j - 1]
                )
            }

        val min1 = endVel1 < endVel2
        if (min1 == lastMin1) {
            disps.add(endDisp)
            if (min1) {
                vels.add(endVel1)
                accels.add(accel1)
            } else {
                vels.add(endVel2)
                accels.add(accel2)
            }
        } else if (accel1 == accel2) {
            disps.add(endDisp)
            vels.add(endVel1)
            accels.add(0.0)
        } else {
            val dx = (endVel2 * endVel2 - endVel1 * endVel1) / (2 * (accel2 - accel1))
            disps.add(endDisp - dx)
            vels.add(sqrt(endVel1 * endVel1 - 2 * accel1 * dx))
            accels.add(max(accel1, accel2))

            disps.add(endDisp)
            vels.add(min(endVel1, endVel2))
            accels.add(min(accel1, accel2))
        }

        lastMin1 = min1
    }

    return DisplacementProfile(disps, vels, accels)
}

fun interface VelConstraint {
    fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double
}

fun interface AccelConstraint {
    fun minMaxProfileAccel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): MinMax
}

class TranslationalVelConstraint(
    @JvmField
    val minTransVel: Double,
) : VelConstraint {
    override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) = minTransVel
}

class AngularVelConstraint(
    @JvmField
    val maxAngVel: Double,
) : VelConstraint {
    override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) =
        maxAngVel / robotPose.rot.velocity().value()
}

class MinVelConstraint(
    @JvmField
    val constraints: List<VelConstraint>,
) : VelConstraint {
    override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) =
        constraints.minOf { it.maxRobotVel(robotPose, path, s) }
}

class ProfileAccelConstraint(
    @JvmField
    val minAccel: Double,
    @JvmField
    val maxAccel: Double,
) : AccelConstraint {
    private val minMax = MinMax(minAccel, maxAccel)

    override fun minMaxProfileAccel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) = minMax
}

class CompositeVelConstraint(
    @JvmField
    val constraints: List<VelConstraint>,
    @JvmField
    val partitions: List<Double>
) : VelConstraint {
    init {
        require(constraints.size == partitions.size + 1)
        // TODO: require ordering?
    }

    override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double {
        for ((offset, constraint) in partitions.zip(constraints.drop(1)).reversed()) {
            if (s >= offset) {
                return constraint.maxRobotVel(robotPose, path, s)
            }
        }

        return constraints.first().maxRobotVel(robotPose, path, s)
    }
}

class CompositeAccelConstraint(
    @JvmField
    val constraints: List<AccelConstraint>,
    @JvmField
    val partitions: List<Double>
) : AccelConstraint {
    init {
        require(constraints.size == partitions.size + 1)
        // TODO: require ordering?
    }

    override fun minMaxProfileAccel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): MinMax {
        for ((offset, constraint) in partitions.zip(constraints.drop(1)).reversed()) {
            if (s >= offset) {
                return constraint.minMaxProfileAccel(robotPose, path, s)
            }
        }

        return constraints.first().minMaxProfileAccel(robotPose, path, s)
    }
}

fun profile(
    path: PosePath,
    beginEndVel: Double,
    velConstraint: VelConstraint,
    accelConstraint: AccelConstraint,
    resolution: Double,
): CancelableProfile {
    val samples = max(1, ceil(path.length() / resolution).toInt())

    val maxVels = mutableListOf<Double>()
    val minAccels = mutableListOf<Double>()
    val maxAccels = mutableListOf<Double>()

    for (s in rangeMiddle(0.0, path.length(), samples)) {
        val pose = path[s, 2]

        maxVels.add(velConstraint.maxRobotVel(pose, path, s))

        val (minAccel, maxAccel) = accelConstraint.minMaxProfileAccel(pose, path, s)
        minAccels.add(minAccel)
        maxAccels.add(maxAccel)
    }

    return profile(
        range(0.0, path.length(), samples + 1),
        beginEndVel, maxVels, minAccels, maxAccels
    )
}

fun forwardProfile(
    path: PosePath,
    beginVel: Double,
    velConstraint: VelConstraint,
    accelConstraint: AccelConstraint,
    resolution: Double,
): DisplacementProfile {
    val samples = max(1, ceil(path.length() / resolution).toInt())

    val maxVels = mutableListOf<Double>()
    val maxAccels = mutableListOf<Double>()

    for (s in rangeMiddle(0.0, path.length(), samples)) {
        val pose = path[s, 2]

        maxVels.add(velConstraint.maxRobotVel(pose, path, s))

        val (_, maxAccel) = accelConstraint.minMaxProfileAccel(pose, path, s)
        maxAccels.add(maxAccel)
    }

    return forwardProfile(
        range(0.0, path.length(), samples + 1),
        beginVel, maxVels, maxAccels,
    )
}

fun backwardProfile(
    path: PosePath,
    velConstraint: VelConstraint,
    endVel: Double,
    accelConstraint: AccelConstraint,
    resolution: Double,
): DisplacementProfile {
    val samples = max(1, ceil(path.length() / resolution).toInt())

    val maxVels = mutableListOf<Double>()
    val minAccels = mutableListOf<Double>()

    for (s in rangeMiddle(0.0, path.length(), samples)) {
        val pose = path[s, 2]

        maxVels.add(velConstraint.maxRobotVel(pose, path, s))

        val (minAccel, _) = accelConstraint.minMaxProfileAccel(pose, path, s)
        minAccels.add(minAccel)
    }

    return backwardProfile(
        range(0.0, path.length(), samples + 1),
        maxVels, endVel, minAccels,
    )
}
