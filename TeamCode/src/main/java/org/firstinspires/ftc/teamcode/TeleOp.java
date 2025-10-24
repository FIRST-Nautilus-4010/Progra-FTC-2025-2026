package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem;

public class TeleOp extends OpMode {

    private MecanumDrive drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    @Override
    public void init() {
        // Inicializa subsistemas y drive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap);

        telemetry.addLine("TeleOp RR + Mecanum listo");
    }

    @Override
    public void loop() {
        // === CONTROL MECANUM CON ROADRUNNER ===
        // Usa sticks del gamepad1 para mover el robot
        double driveY = -gamepad1.left_stick_y;  // Adelante/Atrás
        double driveX = -gamepad1.left_stick_x;  // Lateral
        double turn   = -gamepad1.right_stick_x; // Rotación

        // Actualiza pose estimada
        drive.updatePoseEstimate();

        // Asigna potencia de movimiento con Road Runner
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                driveX,       // avance
                driveY       // desplazamiento lateral
                ),
                turn          // rotación
        ));

        // === SUBSISTEMAS ===
        if (gamepad1.a) {
            intake.take().run(new TelemetryPacket());
            shooter.intake();
        }

        if (gamepad1.b) {
            if (!shooter.prepareForShoot().run(new TelemetryPacket())) {
                intake.shoot();
            }
        }

        // === TELEMETRÍA ===
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
    }
}
