package odometry;

import utility.RotationUtil;
import utility.pose;

//XXX: it seems that the functionality of "representing an odometry wheel" and "odometry calculations" are entangled
// in this class, and weirdly (see: Single Responsibility Principle).
// If I were to design this, I would make an _interface_ that represents a very basic Odometry wheel
// (only getEncoderTicks() and maybe a few others), separate classes that represent simulated/mock and real wheel,
// and a separate class for calculations that uses this interface (dependency injection).
public abstract class OdometryWheel {
    //XXX: Some of these would make sense to be constructor parameters.
    protected double deltaTicks = 0;
    int ticksPerRev = 1024;
    double radius = 3; //Centimeters
    pose offset;
    private double prevTicks = 0;

    // Offset is x and y displacement from center of rotation
    // + angle that wheel is facing (in radians with 0 facing right).
    // Make sure angle faces towards front of robot (unless sideways)
    // Obviously, neither of the two possible angles of a sideways
    // wheel is facing forwards. Just be consistent with sideways wheel angles
    public OdometryWheel(pose offset) {
        this.offset = offset;
    }

    //XXX: naming: getRaw what? encoderTicks?
    abstract long getRaw();

    /**
     * Request and calculate the change in ticks of this odo wheel
     */
    void updateDelta() {
        //XXX: this seems like a weird update order/update data. Why not just "currentTicks" and "deltaTicks"?
        // or, just currentTicks and calculate deltaTicks later?
        prevTicks += deltaTicks;
        //get ticks
        double measurement = getRaw();
        deltaTicks = RotationUtil.turnLeftOrRight(prevTicks, measurement, ticksPerRev);
    }

    /**
     * @return the difference in ticks between the most recent time
     * updateDelta was called and the time before that
     */
    double getDeltaTicks() {
        return deltaTicks;
    }

    double getDeltaPosition() {
        // theta * radians = arc length
        //return -1;
        return (getDeltaTicks() / ticksPerRev) * radius;
    }

    double distanceTraveledTowardsAngle(double deltaPosition, double targetAngle) {
        return deltaPosition / cos(targetAngle - offset.r);
    }

    //effect of the bot trans vector on wheel
    //or WheelVec DOT odoWheelVec
    //inverse of distanceTraveledTowardsAngle
    double dotProduct(double botTransMag, double botTransDir) {
        return botTransMag * Math.cos(botTransDir - offset.r);
    }

    double ccTangentDir(double xCenter, double yCenter) {
        double directionFromCenter = Math.atan2(offset.y - yCenter, offset.x - xCenter);
        return directionFromCenter + Math.PI / 2;
    }

    double arclengthToAngle(double arclength, double xCenter, double yCenter) {
        return arclength / Math.hypot(offset.x - xCenter, offset.y - yCenter);
    }

    double angleToArclength(double angle, double xCenter, double yCenter) {
        return angle * Math.hypot(offset.x - xCenter, offset.y - yCenter);
    }

    /**
     * Change in the odometry wheel's tracked distance converted
     * to radians of rotation around the robot's center of rotation
     *
     * @param deltaPosition odo wheel distance change
     * @param xCenter       center of rotation of robot
     * @param yCenter       center of rotation of robot
     * @return change in robot angle about center of rotation
     */
    double odoDeltaToBotAngle(double deltaPosition, double xCenter, double yCenter) {
        double arclength = distanceTraveledTowardsAngle(
                deltaPosition,
                ccTangentDir(xCenter, yCenter));
        return arclengthToAngle(arclength, xCenter, yCenter);
    }

    double robotAngleToOdoDelta(double angle, double xCenter, double yCenter) {
        double arclength = angleToArclength(angle, xCenter, yCenter);
        return dotProduct(arclength, ccTangentDir(xCenter, yCenter));
    }

    double cos(double v) {
        //if v = π/2 + nπ
        if ((Math.abs(v - Math.PI / 2) % Math.PI) < 0.01)
            return 0;

        return Math.cos(v);
    }

    double distanceToCenter() {
        return Math.hypot(offset.x, offset.y);
    }
}
