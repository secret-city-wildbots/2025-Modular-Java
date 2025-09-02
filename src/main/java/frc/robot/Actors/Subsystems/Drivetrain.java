package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Actors.SwerveModule;
import frc.robot.Actors.SwerveModules;
import frc.robot.States.DrivetrainState;
import frc.robot.Utils.Limelight;
import frc.robot.Utils.LimelightHelpers.PoseEstimate;

public class Drivetrain extends SubsystemBase {

    // dependent on robot
    public double[] maxGroundSpeed_mPs;
    public double[] maxRotateSpeed_radPs;
    public double robotWidth_m;
    public double robotLength_m;
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

    public Drivetrain(Pigeon2 pigeon, Limelight[] limelights, double robotWidth_m, double robotLength_m,
            Translation2d[] swerveModuleLocations_m, double[] maxGroundSpeed_mPs, double[] driveGearRatios,
            double azimuthGearRatio, double wheelRadius_m) {

        // set properties of this drivetrain
        this.swerveModuleLocations_m = swerveModuleLocations_m;
        this.robotWidth_m = robotWidth_m;
        this.robotLength_m = robotLength_m;
        this.maxGroundSpeed_mPs = maxGroundSpeed_mPs;
        this.limelights = limelights;

        // pigeon
        this.pigeon = pigeon;

        imuOffset = pigeon.getRotation2d();

        // this wizardy is needed to create 1 maxRotateSpeed for every maxGroundSpeed
        this.maxRotateSpeed_radPs = new double[maxGroundSpeed_mPs.length];
        for (int i = 0; i < maxGroundSpeed_mPs.length; i++) {
            maxRotateSpeed_radPs[i] = maxGroundSpeed_mPs[i]
                    / ((Math.hypot(robotLength_m, robotWidth_m)));
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

        double currentDriveSpeed_mPs = Math
                .sqrt(Math.pow(currentSpeeds.vxMetersPerSecond, 2) + Math.pow(currentSpeeds.vyMetersPerSecond, 2));

        this.state.updateState(currentDriveSpeed_mPs, getIMURotation());

        poseEstimator.update(getIMURotation(), swerveModules.getPosition());

        if (limelights.length == 1) {
        } else if (limelights.length >= 2) {
            ArrayList<PoseEstimate> posEstimates = new ArrayList<PoseEstimate>();
            for (Limelight limelight : limelights) {
                Optional<PoseEstimate> pos = limelight.getPosEstimate(getIMURotation());
                if (pos.isPresent()) {
                    posEstimates.add(pos.get());
                }
            }

            //get the best estimate if multiple are present, else do the only one
            if (posEstimates.size() >= 2) {
                 PoseEstimate[] posEstimatesArr = posEstimates.toArray(new PoseEstimate[posEstimates.size()]);
                PoseEstimate best = Arrays.stream(posEstimatesArr)
                    .max(Comparator.comparingDouble(o -> o.avgTagDist))
                    .orElse(null);
            }
        }
    }
}