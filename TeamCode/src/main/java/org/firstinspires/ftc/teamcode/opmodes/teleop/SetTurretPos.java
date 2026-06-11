package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "SetTurretPos")
public class SetTurretPos extends NextFTCOpMode {
    public SetTurretPos() {
        addComponents(
                BulkReadComponent.INSTANCE
        );
    }

    public final MotorEx motor1 = new MotorEx("turret1").reversed();

    @Override
    public void onStartButtonPressed() {
        motor1.zero();
    }
}
