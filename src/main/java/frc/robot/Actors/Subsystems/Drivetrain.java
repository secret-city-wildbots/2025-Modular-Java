package frc.robot.Actors.Subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Optional;
import java.util.Vector;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Actors.SwerveModule;
import frc.robot.Actors.SwerveModules;
import frc.robot.States.DrivetrainState;
import frc.robot.Utils.Limelight;
import frc.robot.Utils.LimelightHelpers.PoseEstimate;

public class Drivetrain extends SubsystemBase {

    // dependent on robot
    public double[] maxGroundSpeed_mPs;
    public double[] maxRotateSpeed_radPs;
    public Translation2d[] swerveModuleLocations_m;
    public Limelight[] limelights;

    // big boy objects
    public SwerveModules swerveModules;
    private Pigeon2 pigeon;
    private final SwerveDriveKinematics kinematics;
    public SwerveDrivePoseEstimator poseEstimator;

    // state
    public DrivetrainState state = new DrivetrainState();
    public SwerveModuleState[] moduleStates;

    // random
    private Rotation2d imuOffset = new Rotation2d();

    public Drivetrain(Pigeon2 pigeon, Limelight[] limelights,
            double[] swerveModuleSpacing_in, double[] maxGroundSpeed_mPs, double[] driveGearRatios,
            double azimuthGearRatio, double wheelRadius_m) {

        // set properties of this drivetrain
        this.maxGroundSpeed_mPs = maxGroundSpeed_mPs;

        this.swerveModuleLocations_m = new Translation2d[4];
        this.swerveModuleLocations_m[0] = new Translation2d(Units.inchesToMeters(swerveModuleSpacing_in[0]),
                Units.inchesToMeters(-swerveModuleSpacing_in[1]));
        this.swerveModuleLocations_m[1] = new Translation2d(Units.inchesToMeters(swerveModuleSpacing_in[0]),
                Units.inchesToMeters(swerveModuleSpacing_in[1]));
        this.swerveModuleLocations_m[2] = new Translation2d(Units.inchesToMeters(-swerveModuleSpacing_in[0]),
                Units.inchesToMeters(swerveModuleSpacing_in[1]));
        this.swerveModuleLocations_m[3] = new Translation2d(Units.inchesToMeters(-swerveModuleSpacing_in[0]),
                Units.inchesToMeters(-swerveModuleSpacing_in[1]));

        if (limelights == null) {
            this.limelights = new Limelight[0];
        } else {
            this.limelights = limelights;
        }

        // pigeon
        this.pigeon = pigeon;

        imuOffset = pigeon.getRotation2d();

        // this wizardy is needed to create 1 maxRotateSpeed for every maxGroundSpeed
        this.maxRotateSpeed_radPs = new double[maxGroundSpeed_mPs.length];
        for (int i = 0; i < maxGroundSpeed_mPs.length; i++) {
            maxRotateSpeed_radPs[i] = maxGroundSpeed_mPs[i]
                    / ((Math.hypot(Units.inchesToMeters(swerveModuleSpacing_in[0]),
                            Units.inchesToMeters(swerveModuleSpacing_in[1]))));
        }

        this.swerveModules = new SwerveModules(new SwerveModule[] {
                new SwerveModule(0, driveGearRatios, azimuthGearRatio, wheelRadius_m),
                new SwerveModule(1, driveGearRatios, azimuthGearRatio, wheelRadius_m),
                new SwerveModule(2, driveGearRatios, azimuthGearRatio, wheelRadius_m),
                new SwerveModule(3, driveGearRatios, azimuthGearRatio, wheelRadius_m)
        });

        this.kinematics = new SwerveDriveKinematics(swerveModuleLocations_m);

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getIMURotation(),
                swerveModules.getPosition(),
                new Pose2d());
    }

    /**
     * Gets the current 2d rotation of the pigeon
     * 
     * @return a Rotation2d containing the current angle of the pigeon
     */
    public Rotation2d getIMURotation() {
        return pigeon.getRotation2d().minus(imuOffset);
    }

    public double getMaxGroundSpeed_mPs() {
        return maxGroundSpeed_mPs[swerveModules.getShiftedState()];
    }

    public double getMaxRotateSpeed_radPs() {
        return maxRotateSpeed_radPs[swerveModules.getShiftedState()];
    }

    /**
     * gets the current pose from the swerve odometry.
     * 
     * @return Pose2d
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gives current robot chassis speed.
     * Used for autoBuilder (pathplanner)
     * 
     * @return current chassis speed
     */
    public ChassisSpeeds getCurrentSpeeds() {
        return kinematics.toChassisSpeeds(moduleStates);
    }

    public boolean isTipping() {
        return (Math.abs(pigeon.getPitch().getValueAsDouble()) > 10)
                || (Math.abs(pigeon.getRoll().getValueAsDouble()) > 10);
    }

    /**
     * Resets the odometry to the specified pose.
     * Used for autoBuilder (pathplanner) and dashboard pos setter
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetPose(Pose2d pose) {
        imuOffset = pigeon.getRotation2d();
        imuOffset = imuOffset.minus(pose.getRotation());
        poseEstimator.resetRotation(getIMURotation());
        poseEstimator.resetPose(pose);
        poseEstimator.resetRotation(getIMURotation());
        poseEstimator.resetPose(pose);
    }

    /**
     * resets the IMU offset to the current rotation of the robot
     */
    public void resetIMU() {
        imuOffset = pigeon.getRotation2d();
        poseEstimator.resetRotation(getIMURotation());
        poseEstimator.update(getIMURotation(), swerveModules.getPosition());
    }

    /**
     * Read the sensor values and update the state to allow
     * the current position and rotation to be updated
     */
    public void updateState() {
        moduleStates = swerveModules.getCurrentState();
        ChassisSpeeds currentSpeeds = kinematics.toChassisSpeeds(moduleStates);

        double currentDriveSpeed_mPs = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        this.state.updateState(currentDriveSpeed_mPs, getIMURotation());

        poseEstimator.update(getIMURotation(), swerveModules.getPosition());

        Optional<PoseEstimate> best = Optional.empty();
        if (limelights.length == 1) {
            best = limelights[0].getPosEstimate(getIMURotation());
        } else if (limelights.length >= 2) {
            ArrayList<PoseEstimate> posEstimates = new ArrayList<PoseEstimate>();
            for (Limelight limelight : limelights) {
                Optional<PoseEstimate> pos = limelight.getPosEstimate(getIMURotation());
                if (pos.isPresent()) {
                    posEstimates.add(pos.get());
                }
            }

            // get the best estimate if multiple are present, else do the only one
            if (posEstimates.size() >= 2) {
                PoseEstimate[] posEstimatesArr = posEstimates.toArray(new PoseEstimate[posEstimates.size()]);
                best = Arrays.stream(posEstimatesArr)
                        .min(Comparator.comparingDouble(o -> o.avgTagDist));
            } else if (posEstimates.size() == 1) {
                best = Optional.of(posEstimates.get(0));
            }
        }

        // add the best vision measurement to the pose estimator
        if (best.isPresent()) {
            poseEstimator.addVisionMeasurement(best.get().pose, best.get().timestampSeconds,
                    VecBuilder.fill(0.5, 0.5, 999999));
        }

        double[] loggingState = new double[] {
                moduleStates[0].angle.getRadians(),
                moduleStates[0].speedMetersPerSecond,
                moduleStates[1].angle.getRadians(),
                moduleStates[1].speedMetersPerSecond,
                moduleStates[2].angle.getRadians(),
                moduleStates[2].speedMetersPerSecond,
                moduleStates[3].angle.getRadians(),
                moduleStates[3].speedMetersPerSecond
        };

        SmartDashboard.putNumberArray("realModuleStates", loggingState);
    }

    /**
     * drive the drivetrain at an x, y and h power field relative
     * 
     * @param xpow        power to drive at in field relative x
     * @param ypow        power to drive at in field relative y
     * @param hpow        power to change the heading (spin) at
     * @param loopTime_ms the looptime, used in some of the calculations
     */
    public void drive(double xpow, double ypow, double hpow, double loopTime_ms) {
        double limitedStrafeX = Math.max(-1.0, Math.min(1.0, xpow));
        double limitedStrafeY = Math.max(-1.0, Math.min(1.0, ypow));
        double limitedRotate = Math.max(-1.0, Math.min(1.0, hpow));

        SwerveModuleState[] moduleStateOutputs = kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                        limitedStrafeX * getMaxGroundSpeed_mPs(),
                        limitedStrafeY * getMaxGroundSpeed_mPs(),
                        limitedRotate * getMaxRotateSpeed_radPs(),
                        getIMURotation()), 0.001 * loopTime_ms));

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStateOutputs, getMaxGroundSpeed_mPs());

        SwerveModuleState[] finalModuleStateOutputs = new SwerveModuleState[4];
        finalModuleStateOutputs[0] = moduleStateOutputs[0];
        finalModuleStateOutputs[1] = moduleStateOutputs[1];
        finalModuleStateOutputs[2] = moduleStateOutputs[2];
        finalModuleStateOutputs[3] = moduleStateOutputs[3];

        swerveModules.pushModuleStates(moduleStateOutputs, getMaxGroundSpeed_mPs());
    }
}