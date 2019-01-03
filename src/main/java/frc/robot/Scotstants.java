package frc.robot;

public interface Scotstants {
    /* An array containing the index values of the drive spark ports.
     Sparks, in order, are as follows:
     Front-Right, Front-Left, Back-Right, and finally Back-Left.
    */
    final int[] DRIVE_SPARK_PORT = {0, 0, 0, 0};

    /* An array containing the index values of the [mechanism] spark ports.
    Sparks, in order, are as follows:
    */
    final int[] MECHANISM_SPARK_PORT = {};

    // An enumerated list of the steps within the robot's autonomous progression.
    enum Auto_Progression {FIRST_STEP};

    // An enumerated list of the various starting positions of the robot.
    enum Auto_Position {Default};

    // The autonomatic speed of the robot for straight autonomous motion.
    final int AUTO_DRIVE_SPEED = 0;

    // PID values for autonomous motion.
    final double AUTO_KF = 0;
    final double AUTO_KP = 0;
    final double AUTO_KI = 0;
    final double AUTO_KD = 0;

    // The deadzone value in which the robot will not accept joystick input.
    final int JOYSTICK_DEADZONE = 0;

    /* Variables for the "turn" method (See Drive.java).*/
    // The margin of error allowed for the gyroscope by the turn method
    final double TURN_OFFSET = 0;
    // The minimum allowed speed for turning the robot autonomously.
    final double MIN_TURN_SPEED = 0;
    // The maximum allowed speed for turning the robot autonomously.
    final double MAX_TURN_SPEED = 0;
    // The minimum degree error where turning the robot at MAX_TURN_SPEED is allowed.
    final double MIN_DEGREES_FULL_SPEED = 0;

    // The amount of encoder rotations in a single foot
    final int ROTATIONS_TO_FEET = 0;

    // The maximum velocity input allowed by the drive sparks.
    final int MAXIMUM_DRIVE_SPARK_INPUT = 0;
}