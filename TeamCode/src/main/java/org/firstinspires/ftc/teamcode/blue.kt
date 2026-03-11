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

@TeleOp(name = "Blue TeleOp")
class blue: NextFTCOpMode() {
    init {
        addComponents(
            BindingsComponent,
            BulkReadComponent,
            LoopTimeComponent(),
            PedroComponent(Constants::createFollower)
        )
    }

    val goalPose: Pose = Pose(0.0, 144.0)
    val fwMotor1: MotorEx = MotorEx("fw1")
    val fwMotor2: MotorEx = MotorEx("fw2")
    val fwMotors = MotorGroup(fwMotor1, fwMotor2)
    val fwControl = ControlSystem.builder()
        .velPid(0.0, 0.0, 0.01)
        .basicFF(0.006, 0.0, 0.0)
        .build()
    var fwTarget = 0.0
    var turretTarget = 0.0
    var autoAim: Boolean = true
    var staticAim = false
    var limelightAim = false
    val turretMotor: MotorEx = MotorEx("turret")
    val turretControl = ControlSystem.builder()
        .posPid(0.0135, 0.0, 0.0008)
        .build()
    val intakeMotor: MotorEx = MotorEx("intake")
    val blocker: ServoEx = ServoEx("blocker")
    val blocker_open_pos: Double = 0.5
    val blocker_block_pos: Double = 0.8
    lateinit var limelight: Limelight3A

    // GoBILDA RGB Indicator Light — plugged into a servo port, named "indicator" in config
    // PWM range: 500–2500µs mapped to servo positions 0.0–1.0
    val indicator: ServoEx = ServoEx("indicator")

    // Color positions derived from the 500–2500µs PWM spectrum:
    //   Red   ~500µs  → position 0.00
    //   Blue  ~780µs  → position 0.14  (blue sits just past red in the spectrum)
    //   Green ~1500µs → position 0.50
    // Tweak these if your light shows slightly different colors — the spectrum
    // can vary a little between units. Use telemetry to dial them in on-robot.
    companion object {
        const val LED_RED   = 0.00
        const val LED_GREEN = 0.50
        const val LED_BLUE  = 0.14
        const val LED_OFF   = 0.48  // ~1480µs, lands in a dark/off gap near center
    }

    var lastLedPos = -1.0

    fun setIndicator(pos: Double) {
        if (pos != lastLedPos) {
            indicator.position = pos
            lastLedPos = pos
        }
    }

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose())

        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0)
        limelight.start()

        // autoAim is true by default → start green
        setIndicator(LED_GREEN)

        Gamepads.gamepad1.rightTrigger.greaterThan(0.0).or(Gamepads.gamepad1.leftTrigger.greaterThan(0.0))
            .whenTrue { intakeMotor.power = -Gamepads.gamepad1.rightTrigger.get() + Gamepads.gamepad1.leftTrigger.get() }
        Gamepads.gamepad1.rightBumper
            .whenTrue {
                blocker.position = blocker_open_pos
                intakeMotor.power = 1.0
            }
            .whenFalse {
                blocker.position = blocker_block_pos
                intakeMotor.power = 0.0
            }

        // X = toggle odometry autoAim → GREEN (or off if toggled off)
        Gamepads.gamepad1.x
            .whenBecomesTrue {
                autoAim = !autoAim
                staticAim = false
                limelightAim = false
                setIndicator(if (autoAim) LED_GREEN else LED_OFF)
            }

        // Y = static aim → RED
        Gamepads.gamepad1.y
            .whenBecomesTrue {
                autoAim = false
                staticAim = true
                limelightAim = false
                setIndicator(LED_RED)
            }

        // B = limelight AprilTag aim → BLUE
        Gamepads.gamepad1.b
            .whenBecomesTrue {
                autoAim = false
                staticAim = false
                limelightAim = true
                setIndicator(LED_BLUE)
            }

        val drivetrain = PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX
        )
        drivetrain.schedule()
        Gamepads.gamepad1.dpadUp
            .whenBecomesTrue { follower.pose = Pose() }
    }

    override fun onUpdate() {
        when {
            limelightAim -> updateLimelightAim()
            else -> updateOdometryAim()
        }
        turretMotor.power = turretControl.calculate(turretMotor.state)
    }

    private fun updateOdometryAim() {
        val currPose = follower.pose
        val targetAng = normalizeAngle(Math.toDegrees(Math.atan2(goalPose.y - currPose.y, goalPose.x - currPose.x) - currPose.heading))
        telemetry.addData("Mode", if (autoAim) "Auto Aim (Odometry)" else "Static Aim")
        telemetry.addData("Target Angle (deg)", targetAng)
        telemetry.update()
        val d = currPose.distanceFrom(goalPose)
        turretTarget = (22301.0 / 6480.0) * targetAng
        if (autoAim) {
            fwTarget = 0.0142645 * d * d + 1.26161 * d + 510.88
            fwControl.goal = KineticState(0.0, fwTarget)
            turretControl.goal = KineticState(turretTarget)
            fwMotors.power = fwControl.calculate(fwMotors.state)
        } else if (staticAim) {
            fwTarget = 0.0
            fwControl.goal = KineticState(0.0, fwTarget)
            turretControl.goal = KineticState(turretTarget)
            fwMotors.power = fwControl.calculate(fwMotors.state)
        }
    }

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
            val tx = aprilTag.targetXDegrees
            val pose = aprilTag.robotPoseTargetSpace
            val d = Math.sqrt(pose.position.x * pose.position.x + pose.position.z * pose.position.z) * 39.3701

            turretTarget = (22301.0 / 6480.0) * tx
            fwTarget = 0.0142645 * d * d + 1.26161 * d + 510.88

            fwControl.goal = KineticState(0.0, fwTarget)
            turretControl.goal = KineticState(turretTarget)
            fwMotors.power = fwControl.calculate(fwMotors.state)

            telemetry.addData("Mode", "Limelight AprilTag Aim")
            telemetry.addData("TX (deg)", tx)
            telemetry.addData("Distance (in)", d)
        } else {
            telemetry.addData("Mode", "Limelight AprilTag Aim")
            telemetry.addData("Limelight", "Tag 20 not detected — holding position")
        }
        telemetry.update()
    }

    fun normalizeAngle(ang: Double): Double {
        var a = ang
        if (a < 180) a -= 360.0
        if (a > 180.0) a += 360.0
        return a
    }
}