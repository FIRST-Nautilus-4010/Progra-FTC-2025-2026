package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Actions.PrepareForShoot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem;

@Autonomous(name="AutonomousRed", group="Testing")
public class AutonomousRed extends LinearOpMode {

    private static final double PPG_POS = -11.6;
    private static final double PGP_POS = 12.2;
    private static final double GPP_POS = 36;

    @Override
    public void runOpMode() throws InterruptedException {

        // Inicializa el drive y la posición inicial
        Pose2d startPose = new Pose2d(-24.62992, 19.62992, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        drive.localizer.setPose(startPose);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);

        Pose2d pose = drive.localizer.getPose();

        Action prepareForShoot = shooter.prepareForShoot(() -> -64 - (pose.position.x), () -> (59 -  (pose.position.y)), pose.heading::toDouble,telemetry);
        Action take = intakeSubsystem.take();
        Action shoot = intakeSubsystem.shoot();
        Action stop = intakeSubsystem.stop();

        // Construye la acción (trajectory) del robot
        Action traj = drive.actionBuilder(startPose)

                .stopAndAdd(prepareForShoot)
                .stopAndAdd(shoot)

                // PPG
                .strafeTo(new Vector2d(PPG_POS, 28 + 3))
                .stopAndAdd(take)
                .strafeTo(new Vector2d(PPG_POS, 28 + 3 + 18))
                .stopAndAdd(stop)
                .strafeTo(new Vector2d(-36.3, 31.5))
                .stopAndAdd(prepareForShoot)
                .stopAndAdd(shoot)

                // PGP
                .strafeTo(new Vector2d(PGP_POS, 28 + 3))
                .stopAndAdd(take)
                .strafeTo(new Vector2d(PGP_POS, 28 + 3 + 18))
                .stopAndAdd(stop)
                .strafeTo(new Vector2d(-7.9, 19.9))
                .stopAndAdd(prepareForShoot)
                .stopAndAdd(shoot)

                // GPP
                .strafeTo(new Vector2d(GPP_POS, 28 + 3))
                .stopAndAdd(take)
                .strafeTo(new Vector2d(GPP_POS, 28 + 3 + 18))
                .stopAndAdd(stop)
                .strafeTo(new Vector2d(49.7, 17.4))
                .stopAndAdd(prepareForShoot)
                .stopAndAdd(shoot)

                .build();

        // Espera al inicio del opmode
        waitForStart();
        if (isStopRequested()) return;

        // Ejecuta la acción y actualiza la pose en tiempo real
        while (opModeIsActive()) {
            Actions.runBlocking(traj);
        }
    }
}
