package org.firstinspires.ftc.teamcode.utils;

public class ShotPoint {
    public final double distanceCm;
    public final double hood;    // 0..1
    public final double velocity; // ticks/s

    public ShotPoint(double distanceCm, double hood, double velocity) {
        this.distanceCm = distanceCm;
        this.hood = hood;
        this.velocity = velocity;
    }
}
