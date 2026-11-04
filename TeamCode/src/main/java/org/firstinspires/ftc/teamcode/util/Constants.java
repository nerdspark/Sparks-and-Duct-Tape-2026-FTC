package org.firstinspires.ftc.teamcode.util;

public final class Constants {

    public static class Shoot {
        public static final double exit_height = 0.25; // m
        public static final double eff = 0.99; // 0-1
        public static final double Cl = 0.01; // 0-1
        public static final double wheel_radius = 0.048; // m
        public static final double gear_ratio = 30.0 / 20.0;
        public static final double hood_angle = Math.toRadians(70);
        public static final double ticksPerRev = 28.0;
        public static final double L = 0.1524; // m
    }

    public static class Elements {
        public static double pollen_radius = 0.07112 / 2.0; // m
        public static double pollen_mass = 0.02494758; // kg
    }
}
