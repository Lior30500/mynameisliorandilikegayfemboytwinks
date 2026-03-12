package org.firstinspires.ftc.teamcode

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.components.LoopTimeComponent
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import kotlin.math.*

@TeleOp(name = "Blue TeleOp")
class blue : NextFTCOpMode() {
    init {
        addComponents(
            BindingsComponent,
            BulkReadComponent,
            LoopTimeComponent(),
            PedroComponent(Constants::createFollower)
        )
    }

    // ── Goal & drivetrain ────────────────────────────────────────────────────
    val goalPose: Pose = Pose(0.0, 144.0)

    // ── Flywheel ─────────────────────────────────────────────────────────────
    val fwMotor1: MotorEx = MotorEx("fw1")
    val fwMotor2: MotorEx = MotorEx("fw2")
    val fwMotors = MotorGroup(fwMotor1, fwMotor2)
    val fwControl = ControlSystem.builder()
        .velPid(0.0, 0.0, 0.01)
        .basicFF(0.006, 0.0, 0.0)
        .build()
    var fwTarget = 0.0

    // ── Turret ────────────────────────────────────────────────────────────────
    val turretMotor: MotorEx = MotorEx("turret")
    val turretControl = ControlSystem.builder()
        .posPid(0.0135, 0.0, 0.0008)
        .build()
    var turretTarget = 0.0

    // ── Aim modes ────────────────────────────────────────────────────────────
    var autoAim: Boolean = true
    var staticAim = false
    var limelightAim = false
    var shootOnMove = false

    // ── Shoot-on-move state ──────────────────────────────────────────────────
    private var lastPose: Pose = Pose()
    private var lastPoseTime: Long = 0L
    private var velX = 0.0
    private var velY = 0.0
    private val VEL_ALPHA = 0.7
    private val FLIGHT_A = 0.0015
    private val FLIGHT_B = 0.05
    private val TURRET_DEG_TO_TICKS = 22301.0 / 6480.0

    // ── Intake & blocker ─────────────────────────────────────────────────────
    val intakeMotor: MotorEx = MotorEx("intake")
    val blocker: ServoEx = ServoEx("blocker")
    val blocker_open_pos: Double = 0.5
    val blocker_block_pos: Double = 0.8

    // ── GoBILDA RGB indicator ────────────────────────────────────────────────
    val indicator: ServoEx = ServoEx("indicator")
    companion object {
        const val LED_RED    = 0.00
        const val LED_GREEN  = 0.50
        const val LED_BLUE   = 0.14
        const val LED_YELLOW = 0.25
        const val LED_OFF    = 0.48
    }
    var lastLedPos = -1.0
    fun setIndicator(pos: Double) {
        if (pos != lastLedPos) { indicator.position = pos; lastLedPos = pos }
    }

    // ── Limelight ─────────────────────────────────────────────────────────────
    lateinit var limelight: Limelight3A

    // ─────────────────────────────────────────────────────────────────────────
    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose())
        lastPose = follower.pose
        lastPoseTime = System.nanoTime()

        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0)
        limelight.start()

        setIndicator(LED_GREEN)

        // ── Intake / blocker bindings ─────────────────────────────────────
        Gamepads.gamepad1.rightTrigger.greaterThan(0.0)
            .or(Gamepads.gamepad1.leftTrigger.greaterThan(0.0))
            .whenTrue {
                intakeMotor.power =
                    -Gamepads.gamepad1.rightTrigger.get() + Gamepads.gamepad1.leftTrigger.get()
            }
        Gamepads.gamepad1.rightBumper
            .whenTrue  { blocker.position = blocker_open_pos; intakeMotor.power = 1.0 }
            .whenFalse { blocker.position = blocker_block_pos; intakeMotor.power = 0.0 }

        // ── Aim mode bindings ────────────────────────────────────────────
        // X = odometry auto-aim (GREEN)
        Gamepads.gamepad1.x.whenBecomesTrue {
            autoAim = !autoAim; staticAim = false; limelightAim = false; shootOnMove = false
            setIndicator(if (autoAim) LED_GREEN else LED_OFF)
        }
        // Y = static aim (RED)
        Gamepads.gamepad1.y.whenBecomesTrue {
            autoAim = false; staticAim = true; limelightAim = false; shootOnMove = false
            setIndicator(LED_RED)
        }
        // B = limelight aim (BLUE)
        Gamepads.gamepad1.b.whenBecomesTrue {
            autoAim = false; staticAim = false; limelightAim = true; shootOnMove = false
            setIndicator(LED_BLUE)
        }
        // A = shoot on move (YELLOW)
        Gamepads.gamepad1.a.whenBecomesTrue {
            autoAim = false; staticAim = false; limelightAim = false; shootOnMove = true
            setIndicator(LED_YELLOW)
        }

        // ── Drivetrain ───────────────────────────────────────────────────
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX
        ).schedule()

        Gamepads.gamepad1.dpadUp.whenBecomesTrue { follower.pose = Pose() }
    }

    // ─────────────────────────────────────────────────────────────────────────
    override fun onUpdate() {
        updateVelocity()

        when {
            limelightAim -> updateLimelightAim()
            shootOnMove  -> updateShootOnMove()
            else         -> updateOdometryAim()
        }

        turretMotor.power = turretControl.calculate(turretMotor.state)
    }

    // ── Velocity tracker ─────────────────────────────────────────────────────
    private fun updateVelocity() {
        val now = System.nanoTime()
        val dt = (now - lastPoseTime) / 1e9
        if (dt < 0.005) return

        val pose = follower.pose
        val rawVx = (pose.x - lastPose.x) / dt
        val rawVy = (pose.y - lastPose.y) / dt

        velX = VEL_ALPHA * velX + (1.0 - VEL_ALPHA) * rawVx
        velY = VEL_ALPHA * velY + (1.0 - VEL_ALPHA) * rawVy

        lastPose = pose
        lastPoseTime = now
    }

    // ── Shoot on move ─────────────────────────────────────────────────────────
    private fun updateShootOnMove() {
        val pose = follower.pose

        val dx = goalPose.x - pose.x
        val dy = goalPose.y - pose.y
        val d  = sqrt(dx * dx + dy * dy)

        val flightTime = FLIGHT_A * d + FLIGHT_B

        val virtualGoalX = goalPose.x - velX * flightTime
        val virtualGoalY = goalPose.y - velY * flightTime

        val rawAngle = Math.toDegrees(
            atan2(virtualGoalY - pose.y, virtualGoalX - pose.x) - pose.heading
        )
        val targetAng = normalizeAngle(rawAngle)

        val dvx = virtualGoalX - pose.x
        val dvy = virtualGoalY - pose.y
        val virtualDist = sqrt(dvx * dvx + dvy * dvy)

        val goalUnitX = dx / d.coerceAtLeast(0.001)
        val goalUnitY = dy / d.coerceAtLeast(0.001)
        val radialVel = velX * goalUnitX + velY * goalUnitY
        val effectiveDist = (virtualDist - radialVel * flightTime).coerceAtLeast(0.0)

        fwTarget = 0.0142645 * effectiveDist * effectiveDist + 1.26161 * effectiveDist + 510.88
        turretTarget = TURRET_DEG_TO_TICKS * targetAng

        fwControl.goal     = KineticState(0.0, fwTarget)
        turretControl.goal = KineticState(turretTarget)
        fwMotors.power     = fwControl.calculate(fwMotors.state)

        telemetry.addData("Mode", "Shoot on Move")
        telemetry.addData("Robot Vel (in/s)", "vx=%.1f  vy=%.1f".format(velX, velY))
        telemetry.addData("Flight Time (s)", "%.3f".format(flightTime))
        telemetry.addData("Virtual Dist (in)", "%.1f".format(effectiveDist))
        telemetry.addData("Turret Angle (deg)", "%.1f".format(targetAng))
        telemetry.update()
    }

    // ── Odometry aim ─────────────────────────────────────────────────────────
    private fun updateOdometryAim() {
        val currPose = follower.pose
        val targetAng = normalizeAngle(
            Math.toDegrees(
                atan2(goalPose.y - currPose.y, goalPose.x - currPose.x) - currPose.heading
            )
        )
        val d = currPose.distanceFrom(goalPose)
        turretTarget = TURRET_DEG_TO_TICKS * targetAng

        telemetry.addData("Mode", if (autoAim) "Auto Aim (Odometry)" else "Static Aim")
        telemetry.addData("Target Angle (deg)", targetAng)
        telemetry.update()

        if (autoAim) {
            fwTarget = 0.0142645 * d * d + 1.26161 * d + 510.88
            fwControl.goal     = KineticState(0.0, fwTarget)
            turretControl.goal = KineticState(turretTarget)
            fwMotors.power     = fwControl.calculate(fwMotors.state)
        } else if (staticAim) {
            fwTarget = 0.0
            fwControl.goal     = KineticState(0.0, fwTarget)
            turretControl.goal = KineticState(turretTarget)
            fwMotors.power     = fwControl.calculate(fwMotors.state)
        }
    }

    // ── Limelight aim ─────────────────────────────────────────────────────────
    private fun updateLimelightAim() {
        val result = limelight.latestResult
        if (result == null || !result.isValid) {
            telemetry.addData("Mode", "Limelight AprilTag Aim")
            telemetry.addData("Limelight", "No result — holding position")
            telemetry.update()
            return
        }
        val aprilTag = result.fiducialResults?.firstOrNull { it.fiducialId == 20 }
        if (aprilTag != null) {
            val tx   = aprilTag.targetXDegrees
            val pose = aprilTag.robotPoseTargetSpace
            val d    = sqrt(
                pose.position.x * pose.position.x + pose.position.z * pose.position.z
            ) * 39.3701

            turretTarget = TURRET_DEG_TO_TICKS * tx
            fwTarget     = 0.0142645 * d * d + 1.26161 * d + 510.88

            fwControl.goal     = KineticState(0.0, fwTarget)
            turretControl.goal = KineticState(turretTarget)
            fwMotors.power     = fwControl.calculate(fwMotors.state)

            telemetry.addData("Mode", "Limelight AprilTag Aim")
            telemetry.addData("TX (deg)", tx)
            telemetry.addData("Distance (in)", d)
        } else {
            telemetry.addData("Mode", "Limelight AprilTag Aim")
            telemetry.addData("Limelight", "Tag 20 not detected — holding position")
        }
        telemetry.update()
    }

    // ── Helpers ───────────────────────────────────────────────────────────────
    fun normalizeAngle(ang: Double): Double {
        var a = ang
        if (a < -180.0) a += 360.0
        if (a > 180.0)  a -= 360.0
        return a
    }
}