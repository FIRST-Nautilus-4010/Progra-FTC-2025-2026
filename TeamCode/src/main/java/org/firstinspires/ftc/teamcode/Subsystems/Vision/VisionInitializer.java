package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionInitializer {

    public static class VisionInitResult {
        public double x;        // metros
        public double y;        // metros
        public double heading;  // radianes
    }

    // Offset FÍSICO de la cámara en el robot (modificable)
    public static double CAM_X = 0.10;  // metros
    public static double CAM_Y = 0.00;  // metros
    public static double CAM_Z = 0.20;  // metros
    public static double CAM_PITCH = Math.toRadians(-90); // mirando hacia adelante
    public static double CAM_YAW   = 0;
    public static double CAM_ROLL  = 0;

    public static VisionInitResult initialize(HardwareMap hardwareMap, Telemetry telemetry) {

        // Pose física de la cámara
        Position camPos = new Position(
                DistanceUnit.METER,
                CAM_X, CAM_Y, CAM_Z,
                0
        );

        YawPitchRollAngles camRot = new YawPitchRollAngles(
                AngleUnit.RADIANS,
                CAM_YAW, CAM_PITCH, CAM_ROLL,
                0
        );

        // Procesador AprilTags
        AprilTagProcessor tag = new AprilTagProcessor.Builder()
                .setCameraPose(camPos, camRot)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "logi1"))
                .addProcessor(tag)
                .build();

        telemetry.addLine("Buscando AprilTag...");
        telemetry.update();

        long start = System.currentTimeMillis();

        while (System.currentTimeMillis() - start < 1500) {
            if (tag.getDetections().size() > 0) {

                AprilTagDetection d = tag.getDetections().get(0);

                if (d.robotPose != null) {
                    VisionInitResult r = new VisionInitResult();

                    r.x = d.robotPose.getPosition().x;
                    r.y = d.robotPose.getPosition().y;
                    r.heading = d.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                    portal.close();
                    return r;
                }
            }
        }

        portal.close();
        return null;
    }
}
