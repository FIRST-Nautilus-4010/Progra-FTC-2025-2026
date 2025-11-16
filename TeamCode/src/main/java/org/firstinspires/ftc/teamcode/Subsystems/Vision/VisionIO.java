package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterIO;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionIO {
    private static final boolean USE_WEBCAM = true;

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;
    private final ShooterIO shooterIO;

    private final Position cameraPosition;

    private volatile Pose2dSimple lastRobotPose = null;

    public VisionIO(HardwareMap hw, ShooterIO shooterIO) {
        this.shooterIO = shooterIO;

        // Camera pose en metros y orientaciones en radianes
        this.cameraPosition = new Position(DistanceUnit.METER, 0, 0.1,0.06 , 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, 0, Math.toRadians(-90), 0, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hw.get(WebcamName.class, "Logi1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.FRONT);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    public void resume() {
        try { visionPortal.resumeStreaming(); } catch (Exception ignored) {}
    }

    public void pause() {
        try { visionPortal.stopStreaming(); } catch (Exception ignored) {}
    }

    public void close() {
        try { visionPortal.close(); } catch (Exception ignored) {}
    }

    /**
     * update(): no bloqueante. Toma la primera detección si existe,
     * corrige heading con shooterIO.getYaw() y guarda pose en metros/rad.
     */
    public void update() {
        List<AprilTagDetection> dets = aprilTag.getDetections();
        if (dets == null || dets.isEmpty()) return;

        AprilTagDetection d = dets.get(0);

        double camX = d.robotPose.getPosition().x;
        double camY = d.robotPose.getPosition().y;

        double camYaw = d.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
        double turretYaw = shooterIO.getYaw();

        double offsetY = cameraPosition.y;
        double offsetX = cameraPosition.x;

        double ry = Math.sin(turretYaw) * offsetY + Math.cos(turretYaw) * offsetY;
        double rx = Math.cos(turretYaw) * offsetX - Math.sin(turretYaw) * offsetY;

        double robotX = camX - rx;
        double robotY = camY - ry;

        double correctedHeading = normalize(camYaw + turretYaw);

        lastRobotPose = new Pose2dSimple(robotX, robotY, correctedHeading);
    }

    public Pose2dSimple getLastRobotPose() { return lastRobotPose; }

    private double normalize(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    public static class Pose2dSimple {
        public final double x;
        public final double y;
        public final double heading;
        public Pose2dSimple(double x, double y, double heading) { this.x = x; this.y = y; this.heading = heading; }
        @Override public String toString() { return "Pose2dSimple{x=" + x + ", y=" + y + ", heading=" + heading + "}"; }
    }
}