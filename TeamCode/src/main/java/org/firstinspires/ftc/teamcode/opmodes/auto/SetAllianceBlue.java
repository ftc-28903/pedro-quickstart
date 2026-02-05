package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.AutoStorage;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "SetBlueAlliance")
public class SetAllianceBlue extends NextFTCOpMode {
    public SetAllianceBlue() {

    }

    @Override
    public void onInit() {
        AutoStorage.allianceColor = AllianceColor.BLUE;
    }
}
