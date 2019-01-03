package frc.robot;

/**
 * A class made to take output data from a BNO055 gyroscope and translate it into concise and usable information.
 */
public class Gyroscope {
    // BNO055 Object used for obtaining translatable data.
    public static BNO055 imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS, BNO055.vector_type_t.VECTOR_EULER);
    
    // The current angle offset of the gyroscope.
    private double angleOffset = 0;

    /**
     * Constructs a new Gyroscope object.
     */
    public Gyroscope() {
        // Resets the gyroscope to prepare for use.
        reset();
    }

    /**
	 * Returns the heading in relation to the offset.
	 * @return The angle heading in relation to the angle offset.
	 */
	public double getOffsetHeading() {
		// Returns the remainder of the current angle divided by 180
		return normalizeHeadingVal(getRawHeading() - angleOffset);
	}

	/**
	 * Resets the angle offset to the current heading.
	 */
	public void reset() {
		// Sets angleOffset to the raw heading of the gyro
		angleOffset = getRawHeading();

		// Checks if the angle offset is 360 degrees
		if (angleOffset == 360.0) {
			// If so, set angleOffset to 0
			// This accounts for a problem seen on the field
			angleOffset = 0;
		}
	}

	/**
	 * Gets the normalized heading of the gyroscope without taking the angle offset into account.
	 * @return The raw angle heading.
	 */
	public double getRawHeading() {
		return normalizeHeadingVal(imu.getVector()[0]);
	}
	
	/**
	 * Returns the angle offset.
	 * @return Angle Offset Value
	 */
	public double getOffset() {
		return angleOffset;
	}

	/**
	 * Normalizes an angle heading value to the range of (-180, 180) degrees.
	 * @return
	 */
	private double normalizeHeadingVal(double heading) {
        // Sets the heading to a value between 0 and 360.
        heading = heading % 360;
        
		// Checks if the new modulated heading is greater than 180
		if (heading > 180.0) {
			// If so, set the heading to a negative value greater than -180
			heading = heading - 360;
		}

		// Otherwise, checks if the opposite case is true
		else if (heading <= -180.0) {
			// If so, set the heading to a positive number less than or equal to 180
			heading = heading + 360.0;
		}

		return heading;
	}
}