package org.firstinspires.ftc.teamcode.Subsystems.Intake.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeIO;

public class Shoot implements Action {
    private final IntakeIO io;

    public Shoot(IntakeIO io) {
        this.io = io;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        io.setPwr(-1);
        //io.setBlockState(false);
        if (isFinished()) {
            onEnd();
        }

        return !isFinished();
    }

    public void onEnd() {
        io.setVel(0);
    }

    public boolean isFinished() {
        return false;
    }
}
