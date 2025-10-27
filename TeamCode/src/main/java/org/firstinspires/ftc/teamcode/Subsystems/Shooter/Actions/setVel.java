package org.firstinspires.ftc.teamcode.Subsystems.Shooter.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterIO;

import java.util.function.Supplier;

public class setVel implements Action {
    private final ShooterIO io;

    private final double vel;

    public setVel(ShooterIO io, double vel) {
        this.io = io;
        this.vel = vel;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        io.setVel(vel);
        return !isFinished();
    }

    public boolean isFinished() {
        return io.getVel() == vel;
    }
}
