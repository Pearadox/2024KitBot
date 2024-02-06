package frc.lib.drivers;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Preferences;

/** Add your docs here. */
public class PearadoxSparkMax extends CANSparkMax {
    /**
     * Creates a new CANSparkMax with the necessary configurations.
     * 
     * @param deviceId     The device ID.
     * @param motorType    The motor type (Brushed/Brushless).
     * @param idleMode     The idle mode (kBrake/kCoast).
     * @param currentLimit The current limit.
     * @param isInverted   The invert type of the motor.
     * @param following    The CANSparkMax of the motor this motor should follow
     */
    public PearadoxSparkMax(int deviceId, MotorType motorType, IdleMode idleMode, int currentLimit, boolean isInverted,
            CANSparkMax following, double rampRate) {
        super(deviceId, motorType);
        this.restoreFactoryDefaults();
        this.setSmartCurrentLimit(currentLimit);
        this.setInverted(isInverted);
        this.setIdleMode(idleMode);
        if (following != null) {
            this.follow(following);
        }
        this.setOpenLoopRampRate(rampRate);
        this.burnFlash();
        String key = "Spark " + this.getDeviceId() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }

    /**
     * Creates a new CANSparkMax with the necessary configurations.
     * 
     * @param deviceId     The device ID.
     * @param motorType    The motor type (Brushed/Brushless).
     * @param idleMode     The idle mode (kBrake/kCoast).
     * @param currentLimit The current limit.
     * @param isInverted   The invert type of the motor.
     */
    public PearadoxSparkMax(int deviceId, MotorType motorType, IdleMode idleMode, int currentLimit,
            boolean isInverted) {
        this(deviceId, motorType, idleMode, currentLimit, isInverted, null, 0);
    }
    
    /**
     * Creates a new CANSparkMax with the necessary configurations.
     * 
     * @param deviceId     The device ID.
     * @param motorType    The motor type (Brushed/Brushless).
     * @param idleMode     The idle mode (kBrake/kCoast).
     * @param currentLimit The current limit.
     * @param isInverted   The invert type of the motor.
     */
    public PearadoxSparkMax(int deviceId, MotorType motorType, IdleMode idleMode, int currentLimit,
            boolean isInverted, double rampRate) {
        this(deviceId, motorType, idleMode, currentLimit, isInverted, null, rampRate);
    }

}