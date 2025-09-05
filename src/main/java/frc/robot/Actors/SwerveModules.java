package frc.robot.Actors;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModules {
    public SwerveModule[] swerveModules;

    /**
     * a helper class so that you don't have to be constantly annoyed by arrays
     * 
     * @param modules the swerve modules to incorporate
     */
    public SwerveModules(SwerveModule[] modules) {
        this.swerveModules = modules;
    }

    /**
     * gets the shifted state of the swerve modules
     * 
     * @return the shifted state as an int
     */
    public int getShiftedState() {
        return this.swerveModules[0].shiftedState;
    }

    /**
     * get the positions of all the swerve modules
     * 
     * @return and array containing all the SwerveModulePositions
     */
    public SwerveModulePosition[] getPosition() {
        SwerveModulePosition[] positions = new SwerveModulePosition[this.swerveModules.length];

        for (int i = 0; i < this.swerveModules.length; i++) {
            positions[i] = this.swerveModules[i].getPosition();
        }

        return positions;
    }

    /**
     * sets the output states of all the swerve modules
     */
    public void pushModuleStates(SwerveModuleState[] moduleState, double maxGroundSpeed_mPs) {
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].pushModuleState(moduleState[i], maxGroundSpeed_mPs);
        }
        double[] loggingState = new double[] {
                moduleState[0].angle.getRadians(),
                moduleState[0].speedMetersPerSecond,
                moduleState[1].angle.getRadians(),
                moduleState[1].speedMetersPerSecond,
                moduleState[2].angle.getRadians(),
                moduleState[2].speedMetersPerSecond,
                moduleState[3].angle.getRadians(),
                moduleState[3].speedMetersPerSecond
        };

        SmartDashboard.putNumberArray("moduleStates", loggingState);
    }

    /**
     * get the module state from all the swerve modules
     * 
     * @return and array containing all the SwerveModuleStates
     */
    public SwerveModuleState[] getCurrentState() {
        SwerveModuleState[] positions = new SwerveModuleState[this.swerveModules.length];

        for (int i = 0; i < this.swerveModules.length; i++) {
            positions[i] = this.swerveModules[i].getCurrentState();
        }

        return positions;
    }
}