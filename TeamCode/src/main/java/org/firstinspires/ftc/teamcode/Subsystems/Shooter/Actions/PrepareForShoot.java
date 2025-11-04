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

    private final double targetHeight = 1;
    private final double accel = -9.81;
    private final Telemetry telemetry;

    private double pitch;
    private double yaw;



    public PrepareForShoot(ShooterIO io, Supplier<Double> distanceWithTargetX, Supplier<Double> distanceWithTargetY, Telemetry telemetry) {
        this.io = io;
        this.distanceWithTargetX = distanceWithTargetX;
        this.distanceWithTargetY = distanceWithTargetY;

        this.telemetry = telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        double distance = Math.hypot(distanceWithTargetY.get(), distanceWithTargetX.get());

        double Vx = Math.sqrt(-accel * distance);
        double Vy = Math.sqrt(2 * -accel * targetHeight);

        pitch = Math.atan2(Vy, Vx);
        yaw = Math.atan2(distanceWithTargetY.get(), distanceWithTargetX.get());

        io.setVel(1500);
        io.setYaw(yaw);
        //io.setPitch(pitch);

        telemetry.addData("desiredShooterPitch", pitch);
        telemetry.addData("desiredShooterYaw", yaw);
        telemetry.addData("desiredShooterVel", 1500);
        telemetry.addData("desiredShooterX", distanceWithTargetX.get());
        telemetry.addData("desiredShooterY", distanceWithTargetY.get());

        return !isFinished();
    }

    public boolean isFinished() {
        return Math.abs(io.getVel() - 1500) < 50 && Math.abs(io.getYaw() - yaw) < 0.1;
    }
}
