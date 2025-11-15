package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionInitializer;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.AimHelper;

@TeleOp(name="Test")
public class Test extends OpMode {

    double robotX = 0;
    double robotY = 0;
    double robotHeading = 0;

    @Override
    public void init() {
        VisionInitializer.VisionInitResult r =
                VisionInitializer.initialize(hardwareMap, telemetry);

        if (r != null) {
            robotX = r.x;
            robotY = r.y;
            robotHeading = r.heading;
        }
    }

    @Override
    public void loop() {

        telemetry.addData("Robot X (m)", robotX);
        telemetry.addData("Robot Y (m)", robotY);
        telemetry.addData("Heading (rad)", robotHeading);

        // Ejemplo: apunta al centro del campo
        double aim = AimHelper.computeYawToTarget(
                robotX, robotY, robotHeading,
                0, 0
        );

        telemetry.addData("Yaw error to target", aim);
        telemetry.update();
    }
}
