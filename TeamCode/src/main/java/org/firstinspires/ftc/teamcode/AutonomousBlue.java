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
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionIO;

import java.util.function.Supplier;

@Autonomous(name="AutonomusBlue", group="Testing")
public class AutonomousBlue extends LinearOpMode {

    private static final double PPG_POS = -11.6;
    private static final double PGP_POS = 12.2;
    private static final double GPP_POS = 36;

    public static Supplier<Pose2d> lastPose = () -> new Pose2d(-70+8, -46.6 + 7.4,  -Math.PI / 2);

    private VisionIO vision;

    @Override
    public void runOpMode() throws InterruptedException {

        // Inicializa el drive y la posición inicial
        Pose2d startPose = new Pose2d(-70+8, -46.6 + 7.4,  -Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        drive.localizer.setPose(startPose);

        lastPose = drive.localizer::getPose;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);

        vision = new VisionIO(hardwareMap, shooter.getIO(), telemetry);

        Action prepareForShoot = shooter.prepareForShoot(
                () -> -64 - drive.localizer.getPose().position.x,
                () -> (-59 - drive.localizer.getPose().position.y),
                () -> drive.localizer.getPose().heading.toDouble(),
                () -> vision.getTagBySpecificId(20),
                2.5,
                telemetry

        );
        Action take = intakeSubsystem.take();
        Action shoot = intakeSubsystem.shoot();
        Action stop = intakeSubsystem.stop();

        // Construye la acción (trajectory) del robot
        Action traj = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-30, -28))

                .stopAndAdd(prepareForShoot)
                .waitSeconds(2)
                .stopAndAdd(shoot)

                // PPG
                .strafeToConstantHeading(new Vector2d(PPG_POS, -28 + 3))
                .stopAndAdd(take)
                .stopAndAdd(shooter.intake())
                .strafeToConstantHeading(new Vector2d(PPG_POS, -28 + 3 - 18))
                .stopAndAdd(stop)
                .strafeToConstantHeading(new Vector2d(-36.3, -31.5))
                .stopAndAdd(prepareForShoot)
                .waitSeconds(2)
                .stopAndAdd(shoot)

                // PGP
                .strafeToConstantHeading(new Vector2d(PGP_POS, -28 + 3))
                .stopAndAdd(take)
                .stopAndAdd(shooter.intake())
                .strafeToConstantHeading(new Vector2d(PGP_POS, -28 + 3 - 18))
                .stopAndAdd(stop)
                .strafeToConstantHeading(new Vector2d(-7.9, -19.9))
                .stopAndAdd(prepareForShoot)
                .waitSeconds(2)
                .stopAndAdd(shoot)

                // GPP
                .strafeToConstantHeading(new Vector2d(GPP_POS, -28 + 3))
                .stopAndAdd(take)
                .stopAndAdd(shooter.intake())
                .strafeToConstantHeading(new Vector2d(GPP_POS, -28 + 3 - 18))
                .stopAndAdd(stop)
                .strafeToConstantHeading(new Vector2d(54.3, -12.9))
                .stopAndAdd(prepareForShoot)
                .waitSeconds(2)
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
