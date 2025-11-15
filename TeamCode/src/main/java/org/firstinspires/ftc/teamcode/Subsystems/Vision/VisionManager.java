package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class VisionManager {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;
    private boolean streaming = false;

    // Camera pose class (posición en pulgadas, orientación en radianes)
    public static class CameraMountPose {
        public final double x, y, z;         // pulgadas
        public final double roll, pitch, yaw; // radianes

        public CameraMountPose(double x, double y, double z, double roll, double pitch, double yaw) {
            this.x = x; this.y = y; this.z = z;
            this.roll = roll; this.pitch = pitch; this.yaw = yaw;
        }
    }

    private CameraMountPose cameraPose;

    // Interfaz para obtener yaw de la cámara en radianes
    public interface CameraYawProvider {
        double getYaw(); // debe devolver radianes
    }

    private CameraYawProvider yawProvider;

    public VisionManager() { }

    // Permite inyectar un proveedor de yaw antes de init
    public void setCameraYawProvider(CameraYawProvider provider) {
        this.yawProvider = provider;
    }

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Valores por defecto de posición (ajusta a tu montaje real en pulgadas)
        double defaultX = 0.0;
        double defaultY = 0.0;
        double defaultZ = 0.0;
        double defaultRoll = 0.0;
        double defaultPitch = Math.toRadians(-90); // ejemplo: cámara mirando hacia abajo

        double yaw = 0.0;
        if (yawProvider != null) {
            try {
                yaw = yawProvider.getYaw();
            } catch (Throwable t) {
                yaw = 0.0;
            }
        }

        cameraPose = new CameraMountPose(defaultX, defaultY, defaultZ, defaultRoll, defaultPitch, yaw);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Logi1"));
        builder.setCameraResolution(new Size(1280, 720));
        builder.addProcessor(aprilTagProcessor);

        try {
            visionPortal = builder.build();
            telemetry.addLine("VisionPortal creado");
            visionPortal.resumeStreaming();
            streaming = true;
            telemetry.addLine("Streaming iniciado");
        } catch (Throwable t) {
            streaming = false;
            telemetry.addData("VisionManager init error", t.getMessage());
        }
        telemetry.update();
    }

    // Actualiza detecciones y refresca yaw si el proveedor existe
    public void update() {
        if (aprilTagProcessor != null) {
            detectedTags = aprilTagProcessor.getDetections();
        }
        if (yawProvider != null) {
            try {
                // Mantener x,y,z,roll,pitch y actualizar yaw dinámicamente
                CameraMountPose p = cameraPose;
                cameraPose = new CameraMountPose(p.x, p.y, p.z, p.roll, p.pitch, yawProvider.getYaw());
            } catch (Throwable ignored) { }
        }
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection getById(int id) {
        for (AprilTagDetection d : detectedTags) {
            if (d != null && d.id == id) return d;
        }
        return null;
    }

    public AprilTagDetection chooseBestDetection() {
        AprilTagDetection best = null;
        double bestRange = Double.MAX_VALUE;
        for (AprilTagDetection d : detectedTags) {
            if (d == null || d.metadata == null) continue;
            try {
                if (d.ftcPose != null) return d;
            } catch (Throwable ignored) {}
            try {
                double r = Double.MAX_VALUE;
                if (d.ftcPose != null) r = d.ftcPose.range;
                else r = d.robotPose.getPosition().z;
                if (r < bestRange) {
                    bestRange = r;
                    best = d;
                }
            } catch (Throwable t) {
                if (best == null) best = d;
            }
        }
        return best;
    }

    public void displayDetectionTelemetry(AprilTagDetection d) {
        if (d == null || telemetry == null) return;
        if (d.metadata != null) {
            telemetry.addLine(String.format("==== (ID %d) %s", d.id, d.metadata.name));
            try {
                telemetry.addLine(String.format("FTCPose XYZ: %6.3f %6.3f %6.3f (m)",
                        d.ftcPose.x, d.ftcPose.y, d.ftcPose.z));
                telemetry.addLine(String.format("FTCPose PRY: %6.1f %6.1f %6.1f (deg)",
                        d.ftcPose.pitch, d.ftcPose.roll, d.ftcPose.yaw));
            } catch (Throwable t) {
                telemetry.addLine(String.format("Rel XYZ (inch): %6.1f %6.1f %6.1f",
                        d.robotPose.getPosition().x,
                        d.robotPose.getPosition().y,
                        d.robotPose.getPosition().z));
                telemetry.addLine(String.format("Rel PRY (deg): %6.1f %6.1f %6.1f",
                        d.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        d.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        d.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            }
        } else {
            telemetry.addLine(String.format("==== (ID %d) Unknown", d.id));
            telemetry.addLine(String.format("Center: %6.0f %6.0f (px)", d.center.x, d.center.y));
        }
    }

    public CameraMountPose getCameraPose() {
        return cameraPose;
    }

    public void resumeStreaming() {
        if (visionPortal != null) {
            try {
                visionPortal.resumeStreaming();
                streaming = true;
            } catch (Throwable ignored) {
                streaming = false;
            }
        }
    }

    public void stopStreaming() {
        if (visionPortal != null) {
            try {
                visionPortal.stopStreaming();
            } catch (Throwable ignored) { }
            streaming = false;
        }
    }

    public boolean isStreaming() {
        return streaming;
    }

    public void stop() {
        if (visionPortal != null) {
            try { visionPortal.close(); } catch (Throwable ignored) { }
            visionPortal = null;
            streaming = false;
        }
    }
}

