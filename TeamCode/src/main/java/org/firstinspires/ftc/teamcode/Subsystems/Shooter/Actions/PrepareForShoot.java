package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterIO;

import java.util.function.Supplier;

public class PrepareForShoot implements Action {
    private final ShooterIO io;

    private final Supplier<Double> distanceWithTargetX;
    private final Supplier<Double> distanceWithTargetY;

    private final double targetHeight = 1;
    private final double accel = -9.81;

    private double pitch;
    private double yaw;

    public PrepareForShoot(ShooterIO io, Supplier<Double> distanceWithTargetX, Supplier<Double> distanceWithTargetY) {
        this.io = io;
        this.distanceWithTargetX = distanceWithTargetX;
        this.distanceWithTargetY = distanceWithTargetY;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        double distance = Math.hypot(distanceWithTargetY.get(), distanceWithTargetX.get());

        double Vx = Math.sqrt(-accel * distance);
        double Vy = Math.sqrt(2 * -accel * targetHeight);

        pitch = Math.atan2(Vy, Vx);
        yaw = Math.atan2(distanceWithTargetY.get(), distanceWithTargetX.get());

        io.setVel(3000);
        io.setYaw(yaw);
        io.setPitch(pitch);

        return !isFinished();
    }

    public boolean isFinished() {
        return io.getVel() == 3000 && io.getYaw() == yaw && io.getPitch() == pitch;
    }
}
