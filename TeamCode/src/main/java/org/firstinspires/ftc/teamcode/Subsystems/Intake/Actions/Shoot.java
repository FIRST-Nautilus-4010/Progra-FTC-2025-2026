package org.firstinspires.ftc.teamcode.Subsystems.Intake.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeIO;

public class Shoot implements Action {
    private final IntakeIO io;
    private boolean initialized = false;
    private int iteration = 0;

    private ElapsedTime elapsedTime = new ElapsedTime();

    public Shoot(IntakeIO io) {
        this.io = io;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            if (elapsedTime.milliseconds() > 2000) {
                initialized = true;
                elapsedTime.reset();
            }
        } else {
            if (elapsedTime.milliseconds() < 50) {
                io.setPwr(1);
            } else if (elapsedTime.milliseconds() < 750) {
                io.setPwr(-1);
            } else {
                io.setPwr(0);
                iteration++;
                elapsedTime.reset();
            }
        }
        io.setBlockState(false);
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
