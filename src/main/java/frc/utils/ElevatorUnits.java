package frc.utils;

public class ElevatorUnits {
    public static double inchesToElevatorUnits(double a) {
        return a * 58.5365853658;
    }

    public static double centimetersToElevatorUnits(double a) {
        return a * 0.393701 * 58.5365853658;
    }

    public static double metersToElevatorUnits(double a) {
        return a * 100 * 0.393701 * 58.5365853658;
    }

    public static double kilometersToElevatorUnits(double a) {
        return a * 100000 * 0.393701 * 58.5365853658;
    }

    public static double milimetersToElevatorUnits(double a) {
        return a * 0.1 * 0.393701 * 58.5365853658;
    }
}
