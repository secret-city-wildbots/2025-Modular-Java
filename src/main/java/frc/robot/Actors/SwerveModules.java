package frc.robot.Actors;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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