package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem;

@TeleOp(name="Test", group="Testing")
public class Test extends OpMode {

    private MecanumDrive drive;


    @Override
    public void init() {
        // Inicializa subsistemas y drive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Test RR + Mecanum listo");
    }

    @Override
    public void loop() {
        // === LECTURA DE STICKS ===
        double driveY = -gamepad1.left_stick_x;  // Adelante/Atrás
        double driveX = -gamepad1.left_stick_y;  // Lateral
        double turn   = -gamepad1.right_stick_x; // Rotación

        Pose2d pose = drive.localizer.getPose();

        // === ACTUALIZA POSE ===
        drive.updatePoseEstimate();
        double heading = -pose.heading.toDouble();

        // === CONVERSIÓN FIELD ORIENTED ===
        double rotatedX = driveX * Math.cos(heading) - driveY * Math.sin(heading);
        double rotatedY = driveX * Math.sin(heading) + driveY * Math.cos(heading);

        // === MOVIMIENTO ===
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(rotatedX, rotatedY),
                turn
        ));
/*
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
*/
        // === TELEMETRÍA ===
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
