package frc.robot.Actors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Utils.MotorType;

public class SwerveModule {
    private final int moduleNumber;
    private final double[] driveGearRatios;
    private final double azimuthGearRatio;
    private final double wheelRadius_m;

    private Motor drive;
    private Motor azimuth;

    private double currentDriveSpeed_mPs = 0;
    private double currentAzimuthAngle_rad = 0;
    private int shiftedState = 0;

    public SwerveModule(int moduleNumber, double[] driveGearRatios, double azimuthGearRatio, double wheelRadius_m) {
        this.moduleNumber = moduleNumber;
        this.driveGearRatios = driveGearRatios;
        this.azimuthGearRatio = azimuthGearRatio;
        this.wheelRadius_m = wheelRadius_m;

        this.drive = new Motor(10 + moduleNumber, MotorType.TFX);
        this.azimuth = new Motor(20 + moduleNumber, MotorType.TFX);

        this.azimuth.pid(0.12, 0.0, 0.0);
    }

    /**
     * updates the internal current state of the swerve module.
     * 
     * @return the current swerveModuleState of the module
     */
    public SwerveModuleState getCurrentState() {
        currentAzimuthAngle_rad = Units.rotationsToRadians(azimuth.pos() / azimuthGearRatio);
        currentDriveSpeed_mPs = drive.vel()
                / driveGearRatios[shiftedState] * 2
                * Math.PI
                * wheelRadius_m;

        return new SwerveModuleState(currentDriveSpeed_mPs, new Rotation2d(currentAzimuthAngle_rad));
    }

    /**
     * Returns the position of the drive and azimuth motors
     * 
     * @return A SwerveModulePosition object
     */
    public SwerveModulePosition getPosition() {
        Rotation2d rotation = new Rotation2d(
                (Units.rotationsToRadians(azimuth.pos() / azimuthGearRatio)));
        return new SwerveModulePosition(
                (drive.pos() / driveGearRatios[shiftedState])
                        * (2 * Math.PI * wheelRadius_m),
                rotation);
    }

    /**
     * Gets any faults from the drive and azimuth motors
     * 
     * @return A boolean array with the structure:
     *         <ul>
     *         <li>drive fault,
     *         <li>azimuth fault
     */
    public boolean[] getSwerveFaults() {
        return new boolean[] { drive.isFault(), azimuth.isFault() };
    }

    public void pushModuleState(SwerveModuleState moduleState, double maxGroundSpeed_mPs, boolean calibrateWheels,
            boolean unlockWheels) {
        moduleState.optimize(new Rotation2d(currentAzimuthAngle_rad));

        // Wrapping the angle to allow for "continuous input"
        double minDistance = MathUtil.angleModulus(moduleState.angle.getRadians() - currentAzimuthAngle_rad);
        double normalAzimuthOutput_rot = Units.radiansToRotations(currentAzimuthAngle_rad + minDistance)
                * moduleState.angle.getRadians();

        if (calibrateWheels) {
            azimuth.resetPos(0.0);
        }

        azimuth.setBrake(!unlockWheels); // invert bc unlock != lock

        azimuth.pos(normalAzimuthOutput_rot);

        // Output drive
        double driveOutput = moduleState.speedMetersPerSecond / maxGroundSpeed_mPs;

        driveOutput *= moduleState.angle.minus(new Rotation2d(currentAzimuthAngle_rad)).getCos();

        drive.dc(driveOutput);
    }

    /**
     * Temperature of drive motor
     * 
     * <ul>
     * <li><b>Minimum Value:</b> 0.0
     * <li><b>Maximum Value:</b> 255.0
     * <li><b>Default Value:</b> 0
     * <li><b>Units:</b> ℃
     * </ul>
     * 
     * @return Double temperature in degrees Celcius
     */
    public double getTemp() {
        return drive.getTemp();
    }

}