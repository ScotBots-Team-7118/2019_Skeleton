package frc.robot;

/**
 * A class made to control the autonomous motions of the robot, as well as run it on its proper course during the autonomous phase.
 */
public class Autonomous {
    // Object declaration.
    DriveBase driveBase;
    Gyroscope gyro;

    // Variable Declaration.
    private Scotstants.Auto_Progression currentPhase;
    private Scotstants.Auto_Position selectedPosition = Scotstants.Auto_Position.Default;

    /**
     * Constructs a new autonomous controller object.
     */
    public Autonomous(DriveBase driveBase, Gyroscope gyro) {
        // Reference assignment.
        this.gyro = gyro;
        this.driveBase = driveBase;

        // Reset the autonomous progression.
        reset();
    }

    /**
     * Runs the autonomous program for the game's designated autonomous phase.
     */
    public void run() {
        switch(currentPhase) {
            case FIRST_STEP:
            break;
        }
    }

    /**
     * Resets the autonomous progression and starting position.
     */
    public void reset() {
        currentPhase = Scotstants.Auto_Progression.FIRST_STEP;
        selectedPosition = Scotstants.Auto_Position.Default;
    }

    /**
     * Tells the autonomous class the position of the robot.
     */
    public void setPosition(Scotstants.Auto_Position position) {
        selectedPosition = position;
    }

    /**
     * Advances the autonomous progression to the next step and
     * resets the appropriate sensors.
     * @param step
     */
    public void nextStep(Scotstants.Auto_Progression step) {
        currentPhase = step;
        gyro.reset();
        driveBase.resetEncoders();
    }
}