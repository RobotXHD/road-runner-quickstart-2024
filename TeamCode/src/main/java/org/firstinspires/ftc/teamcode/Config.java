package org.firstinspires.ftc.teamcode;

@com.acmerobotics.dashboard.config.Config
public class Config {
    public static double p = 50, i = 0, d = 2, setpoint = -120, armcoefficient=1, slidercoefficient = 1;
    public static double pstatic = 0.00004, istatic = 0.0, dstatic = 0.0;
    public static double pstaticaut = 0.0009, istaticaut = 0.0, dstaticaut = 0.002;
    public static double py = 0.0005, iy = 0, dy = 0.004, setpointY = 0;
    public static double px = 0.001, ix = 0, dx = 0.006, setpointX = 0;
    public static double sidewaysCalib = 45.8636, rotationCalib = 75.8;
    public static double ps = 0.002, is = 0, ds = 0, setPointS = 0;
    public static double toleranceX = 200, toleranceY = 200, toleranceRotatie = 2, toleranceXPlaca = 350, tolerantaYPlaca = 350, tolerance=200;
    public static int targetVerifications = 15;
}
