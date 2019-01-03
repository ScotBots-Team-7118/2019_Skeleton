package frc.robot;

// Imports for Drive.java.
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;

/**
 * A class representing the actions and properties of the drive train for the robot.
 */
public class DriveBase {
    // Object declaration.
    Spark sparkFR, sparkFL, sparkBR, sparkBL;
    Gyroscope gyro;
    Encoder encR, encL;
    PIDController pidFR, pidFL, pidBR, pidBL;

    /**
     * Constructs a new drive train object with gyroscope input.
     * @param gyro
     */
    public DriveBase(Gyroscope gyro) {
        // Spark initialization.
        sparkFR = new Spark(Scotstants.DRIVE_SPARK_PORT[0]);
        sparkFL = new Spark(Scotstants.DRIVE_SPARK_PORT[1]);
        sparkBR = new Spark(Scotstants.DRIVE_SPARK_PORT[2]);
        sparkBL = new Spark(Scotstants.DRIVE_SPARK_PORT[3]);

        // Encoder Initialization
        encR = new Encoder(0, 0);
        encL = new Encoder(0, 0);

        // PID Controller Initialization.
        pidFR = new PIDController(0, 0, 0, 0, encR, sparkFR);
        pidFL = new PIDController(0, 0, 0, 0, encL, sparkFL);
        pidBR = new PIDController(0, 0, 0, 0, encR, sparkBR);
        pidBL = new PIDController(0, 0, 0, 0, encL, sparkBL);

        // Sets PID Controllers to use the desired values for autonomous mode.
        setPIDConstants(Scotstants.AUTO_KF, Scotstants.AUTO_KP, Scotstants.AUTO_KI, Scotstants.AUTO_KD);

        // Gyroscope reference assignment.
        this.gyro = gyro;

        // Disables PID Control in case the robot is not about to be in autonomous mode.
        disablePID();
    }

    /**
     * Sets the right side of the drive train to a given velocity,
     * provided that 'v' is between -1 and 1.
     * @param v
     */
    public void setRight(double v) {
        // Checks to make sure 'v' is between -1 and 1.
        if (v < -1 || v > 1) {
            // If it isn't, throw the appropriate error message along with a checked exception.
            if (v < -1) throw new IllegalArgumentException("Right-side velocity input may not be less than -1.");
            else throw new IllegalArgumentException("Right-side velocity input may not be greater than 1.");
        }

        // Checks if 'v' is above the maximum allowed forwards input for the drive sparks.
        else if (v > Scotstants.MAXIMUM_DRIVE_SPARK_INPUT) {
            // If so, set the right-side sparks to the maximum allowed forwards input.
            sparkFR.set(Scotstants.MAXIMUM_DRIVE_SPARK_INPUT);
            sparkBR.set(Scotstants.MAXIMUM_DRIVE_SPARK_INPUT);
        }

        // Checks if 'v' is below the maximum allowed backwards input for the drive sparks.
        else if (v < -Scotstants.MAXIMUM_DRIVE_SPARK_INPUT) {
            // If so, set the right-side sparks to the maximum allowed backwards input.
            sparkFR.set(-Scotstants.MAXIMUM_DRIVE_SPARK_INPUT);
            sparkBR.set(-Scotstants.MAXIMUM_DRIVE_SPARK_INPUT);
        }

        else {
            // If there are no problems with 'v', set the right-side sparks to its numerical value.
            sparkFR.set(v);
            sparkBR.set(v);
        }
    }

    public void setLeft(double v) {
        // Checks to make sure 'v' is between -1 and 1.
        if (v < -1 || v > 1) {
            // If it isn't, throw the appropriate error message along with a checked exception.
            if (v < -1) throw new IllegalArgumentException("Left-side velocity input may not be less than -1.");
            else throw new IllegalArgumentException("Left-side velocity input may not be greater than 1.");
        }

        // Checks if 'v' is above the maximum allowed forwards input for the drive sparks.
        else if (v > Scotstants.MAXIMUM_DRIVE_SPARK_INPUT) {
            // If so, set the left-side sparks to the maximum allowed forwards input.
            sparkFL.set(Scotstants.MAXIMUM_DRIVE_SPARK_INPUT);
            sparkBL.set(Scotstants.MAXIMUM_DRIVE_SPARK_INPUT);
        }

        // Checks if 'v' is below the maximum allowed backwards input for the drive sparks.
        else if (v < -Scotstants.MAXIMUM_DRIVE_SPARK_INPUT) {
            // If so, set the left-side sparks to the maximum allowed backwards input.
            sparkFR.set(-Scotstants.MAXIMUM_DRIVE_SPARK_INPUT);
            sparkBR.set(-Scotstants.MAXIMUM_DRIVE_SPARK_INPUT);
        }

        else {
            // If there are no problems with 'v', set the left-side sparks to its numerical value.
            sparkFR.set(v);
            sparkBR.set(v);
        }
    }

    /**
     * Sets the drive sparks on both sides of the robot to a given velocity,
     * provided that velocity is between -1 and 1.
     * @param v
     */
    public void moveStraight(double v) {
        // Sets both the right and left sparks to the desired velocity,
        // assuming the given constraints are met.
        setRight(v);
        setLeft(v);
    }

    /**
     * Stops the robot's directional movement by setting all drive sparks to 0.
     */
    public void stop() {
        setRight(0);
        setLeft(0);
    }

    /**
     * Turns the robot a given amount of degrees autonomously,
     * and slows down using simple linear regression.
     * @param degrees
     * @return Whether or not the robot is finished turning.
     */
    public boolean turn(double angle) {
        // Defaults the speed to the maximum speed
		double speed = Scotstants.MAX_TURN_SPEED;
		// Finds how far off the gyroscope angle heading is from the desired angle (aka angle error).
		double delta = Math.abs(gyro.getOffsetHeading() - angle);

        // Checks if the angle error is below the minimum degree measure where full speed is allowed.
        // If this is true, it means that the speed needs to be adjusted for the robot to turn accurately.
		if (delta <= Scotstants.MIN_DEGREES_FULL_SPEED) {
            /**
             * How the speed adjustment works:
             * Because we want our function's output to be a linear speed [ft/s] and our input is
             * an angle [degrees], the function's slope [m] is in [ft/(s*degrees)]. In order to calculate
             * this, the function uses two ordered pairs: ([Minimum degrees for full speed], [Max speed]),
             * and ([Turn offset/Allowed gyroscope margin of error], [Minimum speed]). With these values,
             * the computer can use the simple m = (Y2 - Y1) / (X2 - X1) equation to find an appropriate
             * linear slope, and then use that slope along with either of the ordered pairs to find b,
             * producing a v = m*(degrees) + b equation for speed adjustment.
             * 
             * NOTE: [Minimum degrees for full speed], [Turn offset], and [Max degrees for above min speed]
             * are all constants. They can be adjusted in the Scotstants.java interface if desired.
             */

            // Calculates 'm' (ft/(degrees*s)).
			double m = ((Scotstants.MAX_TURN_SPEED - Scotstants.MIN_TURN_SPEED) / (Scotstants.MIN_DEGREES_FULL_SPEED - Scotstants.TURN_OFFSET));
            
            // Calculates 'b' through method of b = Y1 - m*X1
			double b = Scotstants.MIN_TURN_SPEED - (m * Scotstants.TURN_OFFSET);

			// Substitutes appropriate values and finds the adjusted speed for known angle "delta".
			speed = m * delta + b;

		}

		// Checks if the gyro angle is less than the desired angle
		if (gyro.getOffsetHeading() < angle - Scotstants.TURN_OFFSET) {
			// If it is, turn left
			setLeft(-speed);
			setRight(speed);
			// and tell the source that turning is not done
			return false;

		}

		// Otherwise, checks if the gyro angle is greater than the desired angle
		else if (gyro.getOffsetHeading() > angle + Scotstants.TURN_OFFSET) {
			// If it is, turn right
			setRight(-speed);
			setLeft(speed);
			// and tell the source that turning is not done
			return false;
		}

		else {
			// If the gyro angle is aligned with the desired angle,
			// tell the source that the robot has turned the desired amount
			stop();
			return true;
		}
    }

    /**
     * Drives the robot using a method for teleoperated control.
     * Intended for a tank drive system - this may be changed later based on
     * how the robot is actually built.
     * @param axisR
     * @param axisL
     */
    public void teleopDrive(double axisR, double axisL) {
        // If the right-side axis value is outside the joystick deadzone,
        // set the right-side drive value to its current value.
        if (Math.abs(axisR) > Scotstants.JOYSTICK_DEADZONE) {
            setRight(axisR);
        }
        // Otherwise, stop the right side of the drive base from moving.
        else setRight(0);

        // If the left-side axis value is outside the joystick deadzone,
        // set the left-side drive value to its current value.
        if (Math.abs(axisL) > Scotstants.JOYSTICK_DEADZONE) {
            setLeft(axisL);
        }
        // Otherwise, stop the left side of the drive base from moving.
        else setLeft(0);
    }

    /**
     * Enables PID control for all of the drive base sparks.
     */
    public void enablePID() {
        pidFR.enable();
        pidFL.enable();
        pidBR.enable();
        pidBL.enable();
    }

    /**
     * Disables PID control for all of the drive base sparks.
     */
    public void disablePID() {
        pidFR.disable();
        pidFL.disable();
        pidBR.disable();
        pidBL.disable();
    }

    /**
     * Sets the constants for PID control to the desired values.
     * @param kF
     * @param kP
     * @param kI
     * @param kD
     */
    public void setPIDConstants(double kF, double kP, double kI, double kD) {
        pidFR.setF(kF);
        pidFL.setF(kF);
        pidBR.setF(kF);
        pidBL.setF(kF);

        pidFR.setP(kP);
        pidFL.setP(kP);
        pidBR.setP(kP);
        pidBL.setP(kP);

        pidFR.setI(kI);
        pidFL.setI(kI);
        pidBR.setI(kI);
        pidBL.setI(kI);

        pidFR.setD(kD);
        pidFL.setD(kD);
        pidBR.setD(kD);
        pidBL.setD(kD);
    }

    /**
     * Resets the encoders on the drive base.
     */
    public void resetEncoders() {
        encR.reset();
        encL.reset();
    }

    /**
     * Calculates the amount of feet the robot has traveled according to the encoders on the drive base.
     * @return The amount of feet traveled by the robot since the last encoder reset.
     */
    public double feetTraveled() {
        double feetRight = encR.get()*Scotstants.ROTATIONS_TO_FEET;
        double feetLeft = encL.get()*Scotstants.ROTATIONS_TO_FEET;
        return (feetRight+feetLeft)/2;
    }
}