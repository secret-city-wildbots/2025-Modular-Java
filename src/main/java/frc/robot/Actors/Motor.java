package frc.robot.Actors;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import frc.robot.Utils.MotorType;

public class Motor {
    public MotorType type;
    public TalonFX motorTFX;
    public TalonFXConfiguration configTFX;
    public Slot0Configs slot0TFX;
    public SparkMax motorSPX;
    public SparkMaxConfig configSPX;
    public int CanID;

    public Motor(int CanID, MotorType type) {
        this.CanID = CanID;
        this.type = type;

        switch (type) {
            case SPX:
                this.motorSPX = new SparkMax(CanID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
                this.configSPX = new SparkMaxConfig();
                break;
            case TFX:
                this.motorTFX = new TalonFX(CanID);
                this.configTFX = new TalonFXConfiguration();
                this.slot0TFX = new Slot0Configs();
                break;
            case None:
                System.err.println("Motor initialized with None type with CanID " + this.CanID);

        }
    }

    /**
     * Sets the duty cycle of the motor
     * 
     * @param dutyCycle from -1.0 to 1.0
     */
    public void dc(double dutyCycle) {
        switch (this.type) {
            case SPX:
                this.motorSPX.set(dutyCycle);
                break;
            case TFX:
                this.motorTFX.set(dutyCycle);
                break;
            case None:
                System.err.println("tried to set dc on None motor with CanID " + this.CanID);
        }
    }

    /**
     * Gets the duty cycle of the motor
     * 
     * @return the duty cycle from -1.0 to 1.0
     */
    public double dc() {
        switch (this.type) {
            case SPX:
                return this.motorSPX.get();
            case TFX:
                return this.motorTFX.get();
            default:
                return 0.0;
        }
    }

    /**
     * Sets the position of the motor without a feedforward
     * 
     * @param pos the position to set to in whatever unit is being used, usually
     *            motor rotations
     */
    public void pos(double pos) {
        switch (this.type) {
            case SPX:
                motorSPX.getClosedLoopController().setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
                break;
            case TFX:
                PositionDutyCycle controlRequest = new PositionDutyCycle(pos);
                controlRequest.FeedForward = 0;
                motorTFX.setControl(controlRequest);
                break;
            case None:
                System.err.println("tried to set pos on None motor with CanID " + this.CanID);
        }
    }

    /**
     * Sets the position of the motor with a feedforward
     * 
     * @param pos the position to set to in whatever unit is being used, usually
     *            motor rotations
     * @param ff  the feedforward
     */
    public void pos(double pos, double ff) {
        switch (this.type) {
            case SPX:
                motorSPX.getClosedLoopController().setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
                break;
            case TFX:
                PositionDutyCycle controlRequest = new PositionDutyCycle(pos);
                controlRequest.FeedForward = ff;
                motorTFX.setControl(controlRequest);
                break;
            case None:
                System.err.println("tried to set pos on None motor with CanID " + this.CanID);
        }
    }

    /**
     * Gets the position of the motor
     * 
     * @return the pos
     */
    public double pos() {
        switch (this.type) {
            case SPX:
                return this.motorSPX.getEncoder().getPosition();
            case TFX:
                return this.motorTFX.getPosition().getValueAsDouble();
            default:
                return 0.0;
        }
    }

    /**
     * Sets the PID of the motor
     * 
     * @param p proportional
     * @param i integral
     * @param d derivative
     */
    public void pid(double p, double i, double d) {
        switch (this.type) {
            case SPX:
                configSPX.closedLoop.pid(p, i, d);
                motorSPX.configure(configSPX, ResetMode.kNoResetSafeParameters,
                        PersistMode.kPersistParameters);
                break;
            case TFX:
                this.slot0TFX.kP = p;
                this.slot0TFX.kI = i;
                this.slot0TFX.kD = d;
                this.motorTFX.getConfigurator().apply(this.slot0TFX);
                break;
            case None:
                System.err.println("tried to set pid on None motor with CanID " + this.CanID);
        }
    }

    public void peakDC(double dc) {
        
    }
}