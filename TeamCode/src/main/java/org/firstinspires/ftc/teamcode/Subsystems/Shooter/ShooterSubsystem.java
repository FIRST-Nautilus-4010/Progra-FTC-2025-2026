package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Actions.PrepareForShoot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Actions.setVel;

import java.util.function.Supplier;

public class ShooterSubsystem {
    private final ShooterIO io;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        io = new ShooterIO(hardwareMap);
    }

    public Action prepareForShoot(Supplier<Double> distanceWithTargetX, Supplier<Double> distanceWithTargetY) {
        return new PrepareForShoot(io, distanceWithTargetX, distanceWithTargetY);
    }

    public Action intake() {
        return new setVel(io, -6000);
    }

    public Action stop() {
        return new setVel(io, 0);
    }

}