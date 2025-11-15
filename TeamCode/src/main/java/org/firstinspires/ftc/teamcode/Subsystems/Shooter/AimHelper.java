package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

public class AimHelper {

    public static double computeYawToTarget(double robotX, double robotY, double robotHeading,
                                            double targetX, double targetY) {

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        double targetAngle = Math.atan2(dy, dx);

        double yawError = targetAngle - robotHeading;

        // Normalizar
        while (yawError > Math.PI) yawError -= 2*Math.PI;
        while (yawError < -Math.PI) yawError += 2*Math.PI;

        return yawError;
    }
}
