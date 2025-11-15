package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * VisionPoseEstimator:
 * - envuelve a VisionInitializer para mantener la última pose detectada
 * - ofrece helpers:
 *    hasPose(), getPose(), getTargetDxMeters(), getTargetDyMeters()
 *
 * Uso:
 *   estimator.estimateAndStore(timeoutMs, turretYawRad, telemetry);
 *   luego getTargetDxMeters() y getTargetDyMeters() para suppliers.
 */
public class VisionPoseEstimator {
    private final VisionInitializer initializer;
    private VisionInitializer.FieldPose lastPose = null;

    // Targets por alianza (ejemplo): azul -> tag 20, rojo -> tag 24.
    // Aquí solo guardamos tagId objetivo; la posición real del tag puede provenir de una tabla externa.
    // Para simplicidad asumimos que la posición XY del tag objetivo ya está conocida (metes la tabla).
    private final TagFieldDatabase tagDb;

    public VisionPoseEstimator(VisionInitializer initializer, TagFieldDatabase tagDb) {
        this.initializer = initializer;
        this.tagDb = tagDb;
    }

    /** Intenta estimar pose (bloqueante hasta timeoutMs). */
    public boolean estimateAndStore(long timeoutMs, double turretYawRad, Telemetry telemetry) {
        VisionInitializer.FieldPose p = initializer.estimateInitialPose(timeoutMs, turretYawRad, telemetry);
        if (p != null) {
            lastPose = p;
            return true;
        }
        return false;
    }

    public boolean hasPose() { return lastPose != null; }
    public VisionInitializer.FieldPose getPose() { return lastPose; }

    /** Determina alliance por Y (m) y devuelve tagId objetivo */
    public int getTargetTagId() {
        if (lastPose == null) return -1;
        return lastPose.y > 0.0 ? tagDb.blueTagId : tagDb.redTagId;
    }

    /** Calcula dx (m) desde robot hasta target (tag). */
    public double getTargetDxMeters() {
        if (lastPose == null) return 0.0;
        int tagId = getTargetTagId();
        TagFieldDatabase.TagInfo info = tagDb.getTag(tagId);
        if (info == null) return 0.0;
        // dx = tagX - robotX (en metros)
        return info.x - lastPose.x;
    }

    /** Calcula dy (m) desde robot hasta target (tag). */
    public double getTargetDyMeters() {
        if (lastPose == null) return 0.0;
        int tagId = getTargetTagId();
        TagFieldDatabase.TagInfo info = tagDb.getTag(tagId);
        if (info == null) return 0.0;
        return info.y - lastPose.y;
    }

    /** Simple database para posiciones de tags en el campo (en metros) */
    public static class TagFieldDatabase {
        public final int blueTagId = 20;
        public final int redTagId = 24;

        // REEMPLAZA estos con posiciones reales en tu campo (en metros).
        public final TagInfo blueTag = new TagInfo(20, 1.0, 3.0);
        public final TagInfo redTag  = new TagInfo(24, -1.0, -3.0);

        public TagInfo getTag(int id) {
            if (id == blueTag.id) return blueTag;
            if (id == redTag.id) return redTag;
            return null;
        }

        public static class TagInfo {
            public final int id;
            public final double x, y;
            public TagInfo(int id, double x, double y) { this.id = id; this.x = x; this.y = y; }
        }
    }
}
