package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Supplier;

public class SubsystemManager {
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final Queue<RobotState> stateQueue;

    // Actions activas para el estado actual (no recrear cada tick)
    private Action intakeAction = null;
    private Action shooterAction = null;
    private RobotState cachedState = null;

    // Opcional: Referencia a un gestor de visión si lo tienes
    // private final VisionManager vision;

    public SubsystemManager(HardwareMap hardwareMap, Telemetry telemetry) {
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        stateQueue = new LinkedList<>();
        // this.vision = new VisionManager(hardwareMap, telemetry); // si usas visión
    }

    public void scheduleState(RobotState state) {
        if (state == null) return;
        stateQueue.add(state);
    }

    /**
     * periodic debe recibir el TelemetryPacket por tick (dashboard) y pasar
     * exactamente ese packet a las Actions que ejecutamos.
     */
    public void periodic(TelemetryPacket packet) {
        if (stateQueue.isEmpty()) return;

        RobotState current = stateQueue.peek();

        // Si entramos en un nuevo estado, inicializamos las Actions asociadas UNA VEZ
        if (cachedState != current) {
            // limpiar cualquier action previa (por seguridad)
            intakeAction = null;
            shooterAction = null;
            cachedState = current;

            switch (current) {
                case TRAVEL:
                    // travel() debe devolver una Action válida (p. ej. SetVel(0) para intake)
                    intakeAction = intake.travel();
                    shooterAction = shooter.stop();
                    break;

                case INTAKE:
                    intakeAction = intake.take();
                    shooterAction = shooter.intake();
                    break;

                case SHOOT:
                    // Proveer Suppliers reales aquí. Ejemplo: lambdas que leen vision o sensores.
                    Supplier<Double> sampleX = () -> {
                        // TODO: reemplazar por VisionManager.getLatestResult().targetX o similar
                        return 0.0;
                    };
                    Supplier<Double> sampleY = () -> {
                        // TODO: reemplazar por VisionManager.getLatestResult().targetY o similar
                        return 0.0;
                    };
                    shooterAction = shooter.prepareForShoot(sampleX, sampleY);
                    // shooterAction prepara y espera; cuando termine, arrancaremos intake.shoot()
                    intakeAction = null;
                    break;
            }
        }

        // Ejecutar Actions activas (si existen) pasando el mismo TelemetryPacket
        boolean shooterRunning = shooterAction != null && shooterAction.run(packet);
        boolean intakeRunning = intakeAction != null && intakeAction.run(packet);

        // Caso especial: en SHOOT, cuando prepareForShoot termina, creamos intake.shoot() una vez
        if (cachedState == RobotState.SHOOT && !shooterRunning && intakeAction == null) {
            intakeAction = intake.shoot();
            intakeRunning = intakeAction != null && intakeAction.run(packet);
        }

        // Si todas las Actions asociadas al estado han terminado, avanzamos en la cola
        boolean anyRunning = shooterRunning || intakeRunning;
        if (!anyRunning) {
            stateQueue.remove();
            cachedState = null;
            shooterAction = null;
            intakeAction = null;
        }
    }
}
