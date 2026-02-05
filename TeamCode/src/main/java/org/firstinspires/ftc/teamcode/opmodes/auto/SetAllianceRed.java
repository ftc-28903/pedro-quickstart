package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.AutoStorage;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "SetRedAlliance")
public class SetAllianceRed extends NextFTCOpMode {
    public SetAllianceRed() {

    }

    @Override
    public void onInit() {
        AutoStorage.allianceColor = AllianceColor.RED;
    }
}
