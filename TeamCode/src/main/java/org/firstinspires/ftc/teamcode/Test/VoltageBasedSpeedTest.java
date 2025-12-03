package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="VoltageBasedSpeedTest", group="test")
public class VoltageBasedSpeedTest extends LinearOpMode {
    Telemetry dashboardTelemetry;


    private DcMotorEx launcherMotorTop;
    private DcMotorEx launcherMotorBottom;

    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        launcherMotorTop = hardwareMap.get(DcMotorEx.class, "launcherTop");
        launcherMotorBottom = hardwareMap.get(DcMotorEx.class, "launcherBottom");
        launcherMotorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotorTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double desiredVoltage = 10.0;
        double batteryVoltage = getBatteryVoltage();
        double power = desiredVoltage / batteryVoltage;





        while (opModeIsActive()) {
            launcherMotorTop.setPower(-power);
            launcherMotorBottom.setPower(power);

            sleep(2000); // esperar a que la velocidad se estabilice

            double measuredVelocityBottom = launcherMotorBottom.getVelocity();
            double measuredVelocityTop = launcherMotorTop.getVelocity();
            dashboardTelemetry.addData("Measured Velocity Bottom", measuredVelocityBottom);
            dashboardTelemetry.addData("Measured Velocity Top", measuredVelocityTop);
            dashboardTelemetry.addData("Battery Voltage", batteryVoltage);
            dashboardTelemetry.addData("Applied Power", power);
            dashboardTelemetry.update();

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