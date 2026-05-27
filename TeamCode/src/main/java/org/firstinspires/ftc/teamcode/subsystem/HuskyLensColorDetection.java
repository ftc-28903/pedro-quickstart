package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class HuskyLensColorDetection implements Subsystem {
    public static final HuskyLensColorDetection INSTANCE = new HuskyLensColorDetection();
    private HuskyLensColorDetection() { }

    public HuskyLens huskyLens;

    @Override
    public void initialize() {
        huskyLens = ActiveOpMode.hardwareMap().get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    @Override
    public void periodic() {

    }
}
