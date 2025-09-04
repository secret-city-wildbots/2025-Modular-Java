package frc.robot.Actors.Subsystems;

// Interanl library imports
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
// External Library imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
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
    public Intake(int moduleNumber) {
        
        // initialize the module number and motor
        this.moduleNumber = moduleNumber;
        this.motor = new Motor(this.moduleNumber, MotorType.TFX);
    }

    /**
     * Sets the intake motor to spin spin in the direction to intake a coral piece
     */
    public void intakeCoral() {
        this.motor.dc(0.2);
        intaking = true;
        outtaking = false;
    }

    /**
     * Sets the intake motor to spin spin in the direction to outtake a coral piece 
     */
    public void outtakeCoral() {
        this.motor.dc(-0.2);
        intaking = false;
        outtaking = true;
    }

    /**
     * Sets the intake motor to spin spin in the direction to intake an algae piece
     */
    public void intakeAlgae() {
        this.motor.dc(-0.4);
        intaking = true;
        outtaking = false;
    }

    /**
     * Sets the intake Fotor to spin spin in the direction to outtake an algae piece 
     */
    public void outtakeAlgae() {
        this.motor.dc(0.4);
        intaking = false;
        outtaking = true;
    }

    /**
     * Sets the intake motor to 0 speed 
     */
    public void stop() {
        this.motor.dc(0.0);
        intaking = false;
        outtaking = false;
    }
    
    /**
     * tells us if we are intaking
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
        // TODO: Create logic using beam break or wtvs
        // if(beam1||beam2||beam3) {
        //   have coral or algae 
        // }
        // else {
        //  no coral or algae
        // }
        return this.hasPiece;
    }

}
