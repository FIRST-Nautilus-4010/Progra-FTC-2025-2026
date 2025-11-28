package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name="VoltageBasedSpeedTest", group="test")
public class VoltageBasedSpeedTest extends LinearOpMode {

    private DcMotorEx launcherMotorTop;
    private DcMotorEx launcherMotorBottom;
    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();

        launcherMotorTop = hardwareMap.get(DcMotorEx.class, "launcherTop");
        launcherMotorBottom = hardwareMap.get(DcMotorEx.class, "launcherBottom");
        double desiredVoltage = 10.0;
        double batteryVoltage = getBatteryVoltage();
        double power = desiredVoltage / batteryVoltage;





        while (opModeIsActive()) {
            launcherMotorTop.setPower(power);
            launcherMotorBottom.setPower(-power);

            sleep(2000); // esperar a que la velocidad se estabilice

            double measuredVelocityBottom = launcherMotorBottom.getVelocity();
            double measuredVelocityTop = launcherMotorTop.getVelocity();
            telemetry.addData("Measured Velocity Bottom", measuredVelocityBottom);
            telemetry.addData("Measured Velocity Top", measuredVelocityTop);
            telemetry.addData("Battery Voltage", batteryVoltage);
            telemetry.addData("Applied Power", power);
            telemetry.update();

        }
    }

    private double getBatteryVoltage() {
        double result = 0;
        for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
            double voltage = sensor.getVoltage();
            if (voltage > result) {
                result = voltage;
            }
        }
        return result;
    }
}