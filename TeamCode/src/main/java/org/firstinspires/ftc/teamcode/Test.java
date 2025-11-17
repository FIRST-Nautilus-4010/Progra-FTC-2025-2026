package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.AllianceDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionIO;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionIO.Pose2dSimple;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterIO;


import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@TeleOp(name="Test", group="Testing")
public class Test extends OpMode {

    private MecanumDrive drive;
    private SubsystemManager subsystemManager;

    private boolean alreadyPressedA = false;
    private boolean alreadyPressedB = false;
    private boolean alreadyPressedX = false;

    private ShooterIO shooter;
    private VisionIO vision;
    private boolean initialPoseSet = false;

    private AllianceDetector allianceDetector;
    private boolean allianceDecided = false;



    @Override
    public void init() {
        // Inicializa subsistemas y drive
        drive = new MecanumDrive(hardwareMap, new Pose2d(-24.62992, 19.62992, Math.PI / 2));
        subsystemManager = new SubsystemManager(hardwareMap, telemetry);

        shooter = new ShooterIO(hardwareMap);
        vision = new VisionIO(hardwareMap, shooter);
        vision.resume();

        drive.localizer.setPose(new Pose2d(0, 0, 0));
        initialPoseSet = false;

        allianceDetector = new AllianceDetector();
        allianceDetector.fieldCenterX = 0.0;
        allianceDetector.thresholdMeters = 0.3;    // zona segura
        allianceDetector.maxSamples = 3;           //muestras

        allianceDecided = false;



        telemetry.addLine("Test RR + Mecanum listo");
        telemetry.addData("status", "init complete");
        telemetry.update();

    }

    @Override
    public void loop() {
        subsystemManager.periodic(drive, new TelemetryPacket());

        vision.update();
        Pose2dSimple vp = vision.getLastRobotPose();
        // === INICIALIZACIÓN DE POSE ===
        if(!initialPoseSet && vp != null){
            drive.localizer.setPose(new Pose2d(vp.x, vp.y, vp.heading));
            initialPoseSet = true;
            try { vision.pause(); } catch (Exception ignored) {}
            telemetry.addData("vision", "initial pose set");
            telemetry.addData("visionX (m)", "%.3f", vp.x);
            telemetry.addData("visionY (m)", "%.3f", vp.y);
            telemetry.addData("visionHeading (deg)", "%.2f", Math.toDegrees(vp.heading));
        } else {
            telemetry.addData("vision", initialPoseSet ? "disabled (initial set)" : "no detection");
        }
        // === DETECCIÓN DE ALIANZA ===
        if (!allianceDecided){
            if (allianceDetector.processPose(vp)){
                allianceDecided = true;
                if (vp != null){
                    drive.localizer.setPose(new Pose2d(vp.x, vp.y, vp.heading));
                }

                try { vision.pause(); } catch (Exception ignored) {}

            }
        }
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

        if (gamepad1.a && !alreadyPressedA) {
            subsystemManager.setState(RobotState.INTAKE);

            alreadyPressedA = true;
        } else {
            alreadyPressedA = false;
        }

        if (gamepad1.b && !alreadyPressedB) {
            subsystemManager.setState(RobotState.SHOOT);
            alreadyPressedB = true;
        } else {
            alreadyPressedB = false;
        }


        if (gamepad1.x && !alreadyPressedX) {
            subsystemManager.setState(RobotState.TRAVEL);
            alreadyPressedX = true;
        } else {
            alreadyPressedX = false;
        }


        // === TELEMETRÍA ===
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("alliance", allianceDecided ? allianceDetector.getAlliance().toString() : "pending");
        telemetry.addData("samples", allianceDetector.getSamples());
        telemetry.addData("red", allianceDetector.getRedCount());
        telemetry.addData("blue", allianceDetector.getBlueCount());

        telemetry.update();
    }

    @Override
    public void stop() {
        try {vision.close();} catch (Exception ignored) {}
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
    }
}