package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.function.Consumer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.ClusterData;
import org.firstinspires.ftc.teamcode.util.Constants.Shoot;
import org.firstinspires.ftc.teamcode.util.Constants.Elements;

public class Shooter {
    private final DcMotorEx shooterLeft, shooterRight, indexer;
    private final ApriltagManager apriltags;
    private State state = State.None;
    private Consumer<Double> command;

    private double target;

    private FtcDashboard dashboard;

    public Shooter(HardwareMap hardwareMap, ApriltagManager apriltags, Consumer<Double> angle, FtcDashboard dash) {
        this.apriltags = apriltags;
        this.dashboard = dash;
        this.command = angle;

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients gains = new PIDFCoefficients(26, 2, 8, 10);
        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, gains);
        shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, gains);


        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        indexer.setVelocityPIDFCoefficients(0,0,0,0);
    }

    public enum State {
        None,
        Spin_Up,
        Align,
        Shoot
    }

    public void setState(State shooterState) {
        state = shooterState;
    }

    public void runState() {
        switch (state) {
            case Spin_Up:
                runPhysics();
                break;
            case Align:
                align();
                break;
            case Shoot:
                runPhysics();
                align();
                shoot();
                break;
            default:
                stop();
                break;
        }
    }

    private void runPhysics() {
        TelemetryPacket packet = new TelemetryPacket();
        ClusterData data = apriltags.getLatestData();
        if (data == null) {
            packet.addLine("Data is null");
            dashboard.sendTelemetryPacket(packet);
            return;
        }
        packet.put("pose x", data.pose.getPosition().x);
        packet.put("pose y", data.pose.getPosition().y);
        packet.put("pose z", data.pose.getPosition().z);
        packet.put("pose yaw", data.pose.getOrientation().getYaw());
        packet.put("pose pitch", data.pose.getOrientation().getPitch());
//        packet.put("pose roll", data.pose.getOrientation().getRoll());
//        packet.put("found", data.percentFound);
//        packet.put("name", data.metadata.name);
        double d = Math.hypot(data.pose.getPosition().x, data.pose.getPosition().y);
//        double deltaH = data.pose.getPosition().z - Shoot.exit_height;
        double deltaH = data.pose.getPosition().z;

//        double entry_angle = Math.atan(Math.tan(Shoot.hood_angle) - (2*deltaH)/d);

        if (d*Math.tan(Shoot.hood_angle) <= deltaH) {
            packet.addLine("Cannot make it (hood)");
            dashboard.sendTelemetryPacket(packet);
            return;
        }

        double K = Math.pow(d, 2) / (2*Math.pow(Math.cos(Shoot.hood_angle), 2) * (d*Math.tan(Shoot.hood_angle) - deltaH));
        double c = (1.2*Math.PI*Math.pow(Elements.pollen_radius, 2)*Shoot.Cl) / (2*Elements.pollen_mass);
        if (c * K >= 1) {
            packet.addLine("Cannot make it (Magnus)");
            dashboard.sendTelemetryPacket(packet);
            return;
        }
        double velo = Math.sqrt((9.81*K)/(1-c*K));
        double motorSpeed = ((2*velo)/(Shoot.eff*Shoot.wheel_radius))*Shoot.gear_ratio*Shoot.ticksPerRev/(2*Math.PI);

        shooterLeft.setVelocity(motorSpeed);
        shooterRight.setVelocity(motorSpeed);
        target = motorSpeed;
        packet.put("Target", motorSpeed);
        packet.put("Left Speed", shooterLeft.getVelocity());
        packet.put("Right Speed", shooterRight.getVelocity());
//        dashboard.sendTelemetryPacket(packet);
    }

    private void align() {
        ClusterData data = apriltags.getLatestData();
        if (data == null) {
            return;
        }
        command.accept(data.pose.getOrientation().getYaw());
    }

    private void shoot() {
        if ((target - shooterLeft.getVelocity()) < 10 && (target - shooterRight.getVelocity()) < 10) {
            indexer.setPower(1);
        }
    }

    private void stop() {
        indexer.setPower(0);
        shooterLeft.setVelocity(0);
        shooterRight.setVelocity(0);
    }
}
