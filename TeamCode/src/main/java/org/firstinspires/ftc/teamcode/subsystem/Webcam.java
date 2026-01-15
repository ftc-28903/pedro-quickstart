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
    public DistanceComponents lastDistanceComponent = new DistanceComponents(0,0);

    private static final double CAMERA_TILT_DEGREES = 15.0; // Camera tilted upwards

    /**
     * Calculates the horizontal distance to an AprilTag, excluding height difference.
     * Takes into account the camera's upward tilt.
     *
     * @param detection The AprilTag detection
     * @return Horizontal distance in cm, or -1 if detection is null
     */
    public double getHorizontalDistance(AprilTagDetection detection) {
        if (detection == null) return -1;

        // Get the range (direct distance) and elevation angle from the detection
        double range = detection.ftcPose.range; // in cm
        double elevationAngle = detection.ftcPose.elevation; // in degrees

        // Adjust elevation angle for camera tilt
        double actualElevationAngle = elevationAngle + CAMERA_TILT_DEGREES;

        // Calculate horizontal distance using cosine
        // horizontalDistance = range * cos(actualElevation)
        double horizontalDistance = range * Math.cos(Math.toRadians(actualElevationAngle));

        return horizontalDistance;
    }

    /**
     * Alternative method: Gets the horizontal distance and height difference separately
     */
    public DistanceComponents getDistanceComponents(AprilTagDetection detection) {
        if (detection == null) return null;

        double range = detection.ftcPose.range;
        double elevationAngle = detection.ftcPose.elevation;
        double actualElevationAngle = elevationAngle + CAMERA_TILT_DEGREES;

        double horizontalDistance = range * Math.cos(Math.toRadians(actualElevationAngle));
        double heightDifference = range * Math.sin(Math.toRadians(actualElevationAngle));

        return new DistanceComponents(horizontalDistance-30, heightDifference);
    }

    // Helper class to return both components
    public static class DistanceComponents {
        public final double horizontal; // cm
        public final double vertical;   // cm

        public DistanceComponents(double horizontal, double vertical) {
            this.horizontal = horizontal;
            this.vertical = vertical;
        }
    }

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
        if(id20 != null && id20.metadata != null) {
            lastDistanceComponent = getDistanceComponents(id20);
        }

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
            ActiveOpMode.telemetry().addLine(String.format(Locale.ENGLISH, "Horizontal: %6.1f, vertical: %6.1f", lastDistanceComponent.horizontal, lastDistanceComponent.vertical));
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
