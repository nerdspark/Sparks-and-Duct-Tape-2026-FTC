package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
@TeleOp(name = "FTC Dashboard Test", group = "Test")
public class Dashboard_Teleop extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private DcMotorEx motorLeft;
    private DcMotorEx motorRight;

    public static double kp = 34.0;
    public static double ki = 1.0;
    public static double kd = 0.0;

    public static int targetVelo = 1000;

    public void init() {
        motorLeft = hardwareMap.get(DcMotorEx.class, "left");
        motorRight = hardwareMap.get(DcMotorEx.class, "right");

        PIDCoefficients gains = new PIDCoefficients(kp, ki, kd);
        motorLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, gains);
        motorRight.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, gains);
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        PIDCoefficients gains = new PIDCoefficients(kp, ki, kd);
        motorLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, gains);
        motorRight.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, gains);

        motorLeft.setVelocity(targetVelo);
        motorRight.setVelocity(targetVelo);

        packet.put("Left Speed", motorLeft.getVelocity());
        packet.put("Right Motor", motorRight.getVelocity());
        packet.put("Target", targetVelo);
        dashboard.sendTelemetryPacket(packet);
    }
}
