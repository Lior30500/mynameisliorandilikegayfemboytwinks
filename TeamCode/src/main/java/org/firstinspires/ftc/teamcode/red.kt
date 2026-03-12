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

@TeleOp(name = "Red TeleOp")
class red: NextFTCOpMode() {
    init {
        addComponents(
            BindingsComponent,
            BulkReadComponent,
            LoopTimeComponent(),
            PedroComponent(Constants::createFollower)
        )
    }

    val goalPose: Pose = Pose(144.0, 144.0)
    val fwMotor1: MotorEx = MotorEx("fw1")
    val fwMotor2: MotorEx = MotorEx("fw2")
    val fwMotors = MotorGroup(fwMotor1, fwMotor2)
    val fwControl = ControlSystem.builder()
        .velPid(0.0, 0.0, 0.0)
        .basicFF(0.0, 0.0, 0.0)
        .build()
    var fwTarget = 0.0
    var turretTarget = 0.0
    var autoAim: Boolean = true
    var staticAim = false
    var limelightAim = false
    val turretMotor: MotorEx = MotorEx("turret")
    val turretControl = ControlSystem.builder()
        .posPid(0.0, 0.0, 0.0)
        .build()
    val intakeMotor: MotorEx = MotorEx("intake")
    val blocker: ServoEx = ServoEx("blocker")
    val blocker_open_pos: Double = 0.0
    val blocker_block_pos: Double = 0.2
    lateinit var limelight: Limelight3A

    // GoBilda RGB Indicator Light (PWM via servo port)
    // PWM pulse range: 500µs–2500µs mapped to 0.0–1.0 by the SDK
    // Color positions per GoBilda spec:
    //   Red   ~600µs  → (600  - 500) / 2000 ≈ 0.05
    //   Green ~1500µs → (1500 - 500) / 2000 = 0.50
    //   Blue  ~2400µs → (2400 - 500) / 2000 = 0.95
    val rgbLight: ServoEx = ServoEx("rgb")
    val RGB_RED   = 0.05
    val RGB_GREEN = 0.50
    val RGB_BLUE  = 0.95

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose())

        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0) // update to your AprilTag pipeline index
        limelight.start()

        // Set initial LED color to match default mode (autoAim = true → green)
        rgbLight.position = RGB_GREEN

        Gamepads.gamepad1.rightTrigger.greaterThan(0.0).or(Gamepads.gamepad1.leftTrigger.greaterThan(0.0))
            .whenTrue { intakeMotor.power = +Gamepads.gamepad1.rightTrigger.get() + Gamepads.gamepad1.leftTrigger.get() }
            .whenTrue { blocker.position = blocker_block_pos }
        Gamepads.gamepad1.rightBumper
            .whenTrue {
                blocker.position = blocker_open_pos
                intakeMotor.power = 1.0
            }
            .whenFalse {
                blocker.position = blocker_block_pos
                intakeMotor.power = 0.0
            }

        // X = toggle odometry autoAim → GREEN
        Gamepads.gamepad1.x
            .whenBecomesTrue {
                autoAim = !autoAim
                staticAim = false
                limelightAim = false
                rgbLight.position = RGB_GREEN
            }

        // Y = static aim → RED
        Gamepads.gamepad1.y
            .whenBecomesTrue {
                autoAim = false
                staticAim = true
                limelightAim = false
                rgbLight.position = RGB_RED
            }

        // B = limelight AprilTag aim → BLUE
        Gamepads.gamepad1.b
            .whenBecomesTrue {
                autoAim = false
                staticAim = false
                limelightAim = true
                rgbLight.position = RGB_BLUE
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
        val d = currPose.distanceFrom(goalPose)
        turretTarget = (358092.0 / 289.0) * targetAng
        if (autoAim) {
            fwTarget = 0.0142645 * d * d + 1.26161 * d + 748.88095
            fwControl.goal = KineticState(0.0, fwTarget)
            turretControl.goal = KineticState(turretTarget)
            fwMotors.power = fwControl.calculate(fwMotors.state)
        } else if (staticAim) {
            fwTarget = 1200.0
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

        val aprilTag = result.fiducialResults?.firstOrNull { it.fiducialId == 24 }

        if (aprilTag != null) {
            val tx = aprilTag.targetXDegrees
            val pose = aprilTag.robotPoseTargetSpace
            val d = Math.sqrt(pose.position.x * pose.position.x + pose.position.z * pose.position.z) * 39.3701

            turretTarget = (358092.0 / 289.0) * tx
            fwTarget = 0.0142645 * d * d + 1.26161 * d + 748.88095

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
        if (a < -180.0) a += 360.0
        if (a > 180.0) a -= 360.0
        return a
    }
}