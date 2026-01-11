package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Webcam implements Subsystem {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    @Override
    public void initialize() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    @Override
    public void periodic() {
        detectedTags = aprilTagProcessor.getDetections();

        AprilTagDetection id20 = getTagBySpecificId(20);
        displayDetectionTelemetry(id20);
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) return;

        if (detectedId.metadata != null) {
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "XYZ %6.1f %6.1f %6.1f (cm)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "PRY %6.1f %6.1f %6.1f (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "RBE %6.1f %6.1f %6.1f (cm, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "\n==== (ID %d) Unknown", detectedId.id));
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "Center %6.0f %6.0f (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public static final Webcam INSTANCE = new Webcam();
    private Webcam() { }
}
