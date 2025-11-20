package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterIO;

import java.util.function.Supplier;

public class PrepareForShoot implements Action {
    private final ShooterIO io;

    private final Supplier<Double> distanceWithTargetX;
    private final Supplier<Double> distanceWithTargetY;
    private final Supplier<Double> botYaw;

    private final double targetHeight = 0.64;
    private double accel = 10.2;
    private final Telemetry telemetry;

    private double pitch;
    private double yaw;
    private double vel;



    public PrepareForShoot(ShooterIO io, Supplier<Double> distanceWithTargetX, Supplier<Double> distanceWithTargetY, Supplier<Double> botYaw, Telemetry telemetry) {
        this.io = io;
        this.distanceWithTargetX = distanceWithTargetX;
        this.distanceWithTargetY = distanceWithTargetY;
        this.botYaw = botYaw;

        this.telemetry = telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        double distance = Math.hypot(distanceWithTargetY.get() * 0.0254,
                distanceWithTargetX.get() * 0.0254);

        double vel = 7;

        if (distance < 2) {
            vel = 5.4;
            accel = 10.1;
        }

        double g   = accel;                 // debería ser 9.81
        double x   = distance;              // distancia horizontal al objetivo
        double y   = targetHeight;          // diferencia de altura

        // ---- ECUACIÓN CORRECTA DEL TIRO PARABÓLICO ----
        // A*T^2 + B*T + C = 0   con T = tan(theta)
        double A = (g * x * x) / (2.0 * vel * vel);
        double B = -x;
        double C = A + y;

        // discriminante
        double discriminant = B*B - 4*A*C;

        double pitch;

        if (discriminant < 0) {
            // No hay solución real: la velocidad no basta
            pitch = Math.toRadians(45);  // fallback
        } else {
            double sqrtD = Math.sqrt(discriminant);

            double T1 = (-B + sqrtD) / (2.0 * A);
            double T2 = (-B - sqrtD) / (2.0 * A);

            double x0 = Math.toRadians(90) - Math.atan(T1);
            double x1 = Math.toRadians(90) - Math.atan(T2);

            // elige la solución positiva
            if (x0 < Math.toRadians(10) || x0 > Math.toRadians(75)) {
                pitch = x1;
            } else if (x1 < Math.toRadians(10) || x1 > Math.toRadians(75)) {
                pitch = x0;
            } else {
                pitch = Math.toRadians(45);
            }
        }

        // ---- CÁLCULO DEL YAW---
        double yaw = Math.atan2(distanceWithTargetY.get() * 0.0254,
                distanceWithTargetX.get() * 0.0254)
                - botYaw.get();

        io.setYaw(yaw);
        io.setPitch(pitch);
        io.setVel((vel * 60) / (0.1016 * Math.PI));

        telemetry.addData("desiredShooterPitch", pitch);
        telemetry.addData("desiredShooterYaw", yaw);
        telemetry.addData("desiredShooterVel", (vel * 60) / (0.1016 * Math.PI));
        telemetry.addData("desiredShooterX", distanceWithTargetX.get() * 0.0254);
        telemetry.addData("desiredShooterY", distanceWithTargetY.get() * 0.0254);

        return !isFinished();
    }

    public boolean isFinished() {
        return Math.abs(io.getVel() - ((vel * 60) / (0.1016 * Math.PI))) < 50;
    }
}
