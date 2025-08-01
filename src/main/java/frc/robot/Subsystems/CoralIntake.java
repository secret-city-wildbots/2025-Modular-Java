package frc.robot.Subsystems;

// Interanl library imports
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
// External Library imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntake extends SubsystemBase {
    private Motor motor;
    private int moduleNumber;
    private boolean hasPiece = false;
    private boolean intaking = false;
    private boolean outtaking = false;

    /**
     * Creates the CoralIntake Class
     * 
     * @param moduleNumber The module number of the intake, used to set the CAN ID
     *                     of the motor
     */
    public CoralIntake(int moduleNumber) {
        // initialize the module number and motor
        this.moduleNumber = moduleNumber;
        this.motor = new Motor(30 + this.moduleNumber, MotorType.TFX);
    }

    /**
     * Sets the intake motor to a certain speed
     * 
     *
     */
    public Command intake() {
        this.intaking = true;
        this.outtaking = false;
        return runOnce(
            () -> this.motor.dc(1.0)
        );

    }

    /**
     * tells us if we are inntaking
     * 
     * @return true if intaking, false otherwise
     */
    public boolean get_intaking() {
        return this.intaking;
    }

    /**
     * tells us if we are outtaking
     * 
     * @return true if outtaking, false otherwise
     */
    public boolean get_outtaking() {
        return this.outtaking;
    }

    /**
     * tells us whether the intake has a coral or not
     * 
     * @return true if the intake has a coral, false otherwise
     */
    public boolean get_hasPiece() {
        return this.hasPiece;
    }

}
