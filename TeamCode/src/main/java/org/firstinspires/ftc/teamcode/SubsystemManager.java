package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem;

import java.util.LinkedList;
import java.util.Queue;

public class SubsystemManager {
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    private final Queue<RobotState> stateQueue;

    private Action intakeAction = null;
    private Action shooterAction = null;

    public SubsystemManager(HardwareMap hardwareMap, Telemetry telemetry) {

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, telemetry);

        stateQueue = new LinkedList<>();
    }

    public void scheduleState(RobotState state) {
        stateQueue.add(state);
    }

    public void periodic() {
        switch (stateQueue.peek()){
            case TRAVEL:
                intakeAction = intake.travel();
                shooterAction = shooter.stop();

                if (!shooterAction.run() && !intakeAction.run()) {
                    stateQueue.remove();
                }
                break;
            case SHOOT:
                shooterAction = shooter.prepareForShoot();
                if (!shooterAction.run()){
                    intakeAction = intake.shoot();
                    if (!intakeAction.run()) {
                        stateQueue.remove();
                    }
                }
                break;
            case INTAKE:
                shooterAction = shooter.intake();
                intakeAction = intake.take();

                if (!shooterAction.run() && !intakeAction.run()) {
                    stateQueue.remove();
                }
                break;
        }
        

    }
}
