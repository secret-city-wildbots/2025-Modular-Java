package frc.robot.Actors;

import com.revrobotics.RelativeEncoder;

import frc.robot.Utils.MotorType;

public class SwerveModule {
    private final int moduleNumber;
    private final double[] driveGearRatios;
    private final double azimuthGearRatio;
    private final double wheelRadius_m;

    private Motor drive;
    private Motor azimuth;

    private double currentDriveSpeed_mPs = 0;
    private double azimuthAngle_rad = 0;

    public SwerveModule(int moduleNumber, double[] driveGearRatios, double azimuthGearRatio, double wheelRadius_m) {
        this.moduleNumber = moduleNumber;
        this.driveGearRatios = driveGearRatios;
        this.azimuthGearRatio = azimuthGearRatio;
        this.wheelRadius_m = wheelRadius_m;

        this.drive = new Motor(10 + moduleNumber, MotorType.TFX);
        this.azimuth = new Motor(20 + moduleNumber, MotorType.TFX);

        this.azimuth.pid(0.12, 0.0, 0.0);
    }

}