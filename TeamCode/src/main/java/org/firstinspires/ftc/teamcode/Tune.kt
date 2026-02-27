package org.firstinspires.ftc.teamcode.Systems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.components.LoopTimeComponent
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx

@Configurable
@TeleOp(name="Shooter Tuning")
class Tune: NextFTCOpMode() {
    init {
        addComponents(
            BindingsComponent,
            BulkReadComponent,
            LoopTimeComponent()
        )
    }
    companion object {
        @JvmField
        var flywheelTarget: Double = 0.0
        @JvmField
        var turretTarget: Double = 0.0

        @JvmField
        var fwPID = PIDCoefficients(0.0,0.0,0.0)
        @JvmField
        var fwFF = BasicFeedforwardParameters(0.0,0.0,0.0)
        @JvmField
        var turretPID = PIDCoefficients(0.0,0.0,0.0)

        var fwControl = ControlSystem.builder()
            .velPid(fwPID)
            .basicFF(fwFF)
            .build()
        var turretControl = ControlSystem.builder()
            .posPid(turretPID)
            .build()
    }

    val turretMotor: MotorEx = MotorEx("")
    val fwMotor1: MotorEx = MotorEx("")
    val fwMotor2: MotorEx = MotorEx("")
    val fwMotors = MotorGroup(fwMotor1,fwMotor2)

    override fun onUpdate() {
        fwControl.goal = KineticState(0.0,flywheelTarget)
        turretControl.goal = KineticState(turretTarget)

        fwMotors.power = fwControl.calculate(fwMotors.state)
        turretMotor.power = turretControl.calculate(turretMotor.state)
    }
}
