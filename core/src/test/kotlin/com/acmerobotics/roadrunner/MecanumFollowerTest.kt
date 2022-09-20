package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.markers.None
import org.knowm.xchart.style.theme.MatlabTheme
import kotlin.math.PI
import kotlin.random.Random

// TODO: make a proper test by mocking out drive hardware
class MecanumFollowerTest {
    @Test
    fun testTimeFollower() {
        val dt = 1 / 25.0

        val kinematics = MecanumKinematics(3.0)

        val path = TangentPath(
            PositionPathBuilder(
                Vector2d(0.0, 0.0),
                Rotation2d.exp(0.0),
                1e-6,
            )
                .splineTo(
                    Vector2d(15.0, 15.0),
                    Rotation2d.exp(PI),
                )
                .splineTo(
                    Vector2d(5.0, 35.0),
                    Rotation2d.exp(PI / 3),
                )
                .build(),
            0.0,
        )

        val profile = TimeProfile(
            profile(
                path,
                0.0,
                // TODO: angular velocity constraint
                kinematics.WheelVelConstraint(50.0),
                ProfileAccelConstraint(-25.0, 25.0),
                0.25,
            ).baseProfile
        )

        val follower = HolonomicController(10.0, 10.0, 0.1)

        val ff = MotorFeedforward(0.0, 1 / 60.0, 0.0)

        var poseEstimate = Pose2d(
            Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0),
        )

        val targetPositions = mutableListOf<Vector2d>()
        val actualPositions = mutableListOf<Vector2d>()

        var t = 0.0
        while (t <= profile.duration) {
            t += dt

            val s = profile[t]
            val targetPose = path[s.value(), 3].reparam(s)

            targetPositions.add(targetPose.trans.value())
            actualPositions.add(poseEstimate.trans)

            val command = follower.compute(
                targetPose,
                poseEstimate,
                Twist2d(
                    Vector2d(0.0, 0.0),
                    0.0,
                )
            )

            val wheelVoltages = kinematics.inverse(command)
//                fun all() = listOf(frontLeft, frontRight, backLeft, backRight)
                .all()
                .map { ff.compute(it) }
                .map { clamp(it * (1.0 + 0.3 * (Random.Default.nextDouble() - 0.5)), -1.0, 1.0) }

            val posDeltas = wheelVoltages
                .map { 60.0 * it * dt }

            // TODO: should this be a list?
            // maybe there should just be a list secondary ctor
            val wheelIncrements = MecanumKinematics.WheelIncrements<Time>(
                DualNum.constant(posDeltas[0], 2),
                DualNum.constant(posDeltas[1], 2),
                DualNum.constant(posDeltas[2], 2),
                DualNum.constant(posDeltas[3], 2),
            )

            poseEstimate += kinematics.forward(wheelIncrements).value()
        }

        val graph = XYChart(600, 400)
        graph.title = "Mecanum PIDVA Follower Sim"
        graph.addSeries(
            "Target Trajectory",
            targetPositions.map { it.x }.toDoubleArray(),
            targetPositions.map { it.y }.toDoubleArray()
        )
        graph.addSeries(
            "Actual Trajectory",
            actualPositions.map { it.x }.toDoubleArray(),
            actualPositions.map { it.y }.toDoubleArray()
        )
        graph.seriesMap.values.forEach { it.marker = None() }
        graph.styler.theme = MatlabTheme()
        saveChart("mecanumTimeFollower", graph)
    }

    @Test
    fun testDispFollower() {
        val dt = 1 / 25.0

        val kinematics = MecanumKinematics(3.0)

        val path = TangentPath(
            PositionPathBuilder(
                Vector2d(0.0, 0.0),
                Rotation2d.exp(0.0),
                1e-6,
            )
                .splineTo(
                    Vector2d(15.0, 15.0),
                    Rotation2d.exp(PI),
                )
                .splineTo(
                    Vector2d(5.0, 35.0),
                    Rotation2d.exp(PI / 3),
                )
                .build(),
            0.0,
        )

        val profile =
            profile(
                path,
                5.0,
                // TODO: angular velocity constraint
                kinematics.WheelVelConstraint(50.0),
                ProfileAccelConstraint(-25.0, 25.0),
                0.25,
            ).baseProfile

        val follower = HolonomicController(10.0, 10.0, 0.1)

        val ff = MotorFeedforward(0.0, 1 / 60.0, 0.0)

        var poseEstimate =
            Pose2d(
                Vector2d(1.0, 0.0),
                Rotation2d.exp(0.0),
            )

        var poseVelocity =
            Twist2d(
                Vector2d(10.0, 0.0),
                0.0,
            )

        val targetPositions = mutableListOf<Vector2d>()
        val actualPositions = mutableListOf<Vector2d>()

        var s = 0.0
        var i = 0
        while (s < profile.length) {
            if (i++ >= 1000) {
                break
            }

            s = project(path, poseEstimate.trans, s)
            val targetPose = path[s, 3]

            println(s)
            println(targetPose)

            val r = targetPose.trans.reparam(profile[s])
            println(r)
            val x =
                Vector2dDual<Time>(
                    DualNum(doubleArrayOf(poseEstimate.trans.x, poseVelocity.transVel.x)),
                    DualNum(doubleArrayOf(poseEstimate.trans.y, poseVelocity.transVel.y)),
                )

            val d = x - r
            val drds = r.drop(1)
            val d2rds2 = drds.drop(1)

            println(x.drop(1) dot drds)
            println((d dot d2rds2))

            val dsdt = (x.drop(1) dot drds) / (-(d dot d2rds2) + 1.0)
            println(dsdt.value())

            // val sDual = DualNum<Time>(doubleArrayOf(s, dsdt[0], 0.0))
            val sDual = DualNum<Time>(doubleArrayOf(s, profile[s][1], 0.0))

            targetPositions.add(targetPose.trans.value())
            actualPositions.add(poseEstimate.trans)

            val command = follower.compute(
                targetPose.reparam(sDual),
                poseEstimate,
                poseEstimate.rot.inverse() * poseVelocity,
            )

            val wheelVoltages = kinematics.inverse(command)
//                fun all() = listOf(frontLeft, frontRight, backLeft, backRight)
                .all()
                .map { ff.compute(it) }
                .map { clamp(it * (1.0 + 0.3 * (Random.Default.nextDouble() - 0.5)), -1.0, 1.0) }

            val posDeltas = wheelVoltages
                .map { 60.0 * it * dt }

            // TODO: should this be a list?
            // maybe there should just be a list secondary ctor
            val wheelIncrements = MecanumKinematics.WheelIncrements<Time>(
                DualNum.constant(posDeltas[0], 2),
                DualNum.constant(posDeltas[1], 2),
                DualNum.constant(posDeltas[2], 2),
                DualNum.constant(posDeltas[3], 2),
            )

            val incrDual = kinematics.forward(wheelIncrements)
            poseEstimate += incrDual.value()
            // TODO: which angle do you use to transform the velocity?
            poseVelocity = poseEstimate.rot * incrDual.velocity().value()
        }

        val graph = XYChart(600, 400)
        graph.title = "Mecanum PIDVA Follower Sim"
        graph.addSeries(
            "Target Trajectory",
            targetPositions.map { it.x }.toDoubleArray(),
            targetPositions.map { it.y }.toDoubleArray()
        )
        graph.addSeries(
            "Actual Trajectory",
            actualPositions.map { it.x }.toDoubleArray(),
            actualPositions.map { it.y }.toDoubleArray()
        )
        graph.seriesMap.values.forEach { it.marker = None() }
        graph.styler.theme = MatlabTheme()
        saveChart("mecanumDispFollower", graph)
    }
}
