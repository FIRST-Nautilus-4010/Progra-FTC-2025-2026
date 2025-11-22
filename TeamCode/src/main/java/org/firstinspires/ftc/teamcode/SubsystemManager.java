package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
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
    private RobotState cachedState = null;

    private final Telemetry telemetry;

    public SubsystemManager(HardwareMap hardwareMap, Telemetry telemetry) {
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, telemetry);

        stateQueue = new LinkedList<>();
        this.telemetry = telemetry;
    }

    public void scheduleState(RobotState state) {
        if (state != null) stateQueue.add(state);
    }

    public void setState(RobotState state) {
        if (state == null) return;
        stateQueue.clear();
        stateQueue.add(state);
    }


    public void periodic(MecanumDrive drive, TelemetryPacket packet, int allianceMult) {

        if (stateQueue.isEmpty()) scheduleState(RobotState.TRAVEL);

        RobotState current = stateQueue.peek();

        if (cachedState != current) {

            intakeAction = null;
            shooterAction = null;
            cachedState = current;

            switch (current) {

                case TRAVEL:
                    intakeAction = intake.stop();
                    shooterAction = shooter.stop();
                    break;

                case INTAKE:
                    intakeAction = intake.take();
                    shooterAction = shooter.intake();
                    break;

                case SHOOT: {

                    double goalX = (allianceMult == 1) ? -64 : 64;
                    double goalY = (allianceMult == 1) ? 59 : -59;

                    shooterAction =
                            shooter.prepareForShoot(
                                    () -> goalX - drive.localizer.getPose().position.x,
                                    () -> goalY - drive.localizer.getPose().position.y,
                                    () -> drive.localizer.getPose().heading.toDouble(),
                                    telemetry
                            );

                    intakeAction = null;   // intake dispara DESPUÉS de preparar
                    break;
                }
            }
        }

        boolean shooterRunning = shooterAction != null && shooterAction.run(packet);
        boolean intakeRunning  = intakeAction  != null && intakeAction.run(packet);



        if (cachedState == RobotState.SHOOT && !shooterRunning && intakeAction == null) {
            intakeAction = intake.shoot();
            intakeRunning = intakeAction.run(packet);
        }

        if (intakeAction == null) {
            intake.stop().run(packet);
        }


        if (!(shooterRunning || intakeRunning)) {
            stateQueue.remove();
            cachedState = null;
            shooterAction = null;
            intakeAction = null;
        }


        intake.periodic();
        shooter.periodic(telemetry);

        telemetry.addData("State", current.toString());
    }
}
