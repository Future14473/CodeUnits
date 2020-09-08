package odometry;

import utility.Average;
import utility.Timing;
import utility.point;
import utility.pose;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

public class Odometry {
    //XXX: I see this uses the "standard" orientation.
    // It might make more sense to use what is called the North-West-Up
    // orientation: +x is forward, +y is left, (+z is up) -- (rotate the standard axis).
    // This way, 0 degrees is intuitively forward.

    // constants
    final static double facingForward = Math.PI / 2;
    final static double facingRight = 0;

    // settings
    List<OdometryWheel> wheels;
    double xCenterOfRotation = 0;
    double yCenterOfRotation = 0;
    // XXX: volatile doesn't solve all concurrency problems, AtomicReference is more flexible -- ill check for potential
    //  issues
    volatile pose position;


    // XXX: It is generally a bad idea to integrate both concurrency and functionality in the same class. This is
    //   not very flexible, and very error prone, and is tightly coupled.
    //   Usually, you should just provide a method(s) that a thread can call, and handle thread logic elsewhere.
    //   In fact, in most modern concurrency schemes, people don't make their own Threads
    //   I can draft up a different version if you like

    // Thread state
    volatile boolean running = false;
    private final Thread loop = new Thread(() -> {
        while (running) {
            wheels.forEach(OdometryWheel::updateDelta);
            position.translateRelative(curvedTrajectoryTranslation(getDeltaPose()));
            Timing.delay(1);
        }
    });

    //XXX: having "default" overloads be _last_ is generally good
    public Odometry(pose initial, List<OdometryWheel> wheels) {
        this.position = initial;
        this.wheels = wheels;
    }

    public Odometry(List<OdometryWheel> wheels) {
        this(new pose(0, 0, 0), wheels);
    }

    //Threading
    public void start() {
        running = true;
        loop.start();
    }

    public void end() {
        running = false;
    }
    //

    public pose getPosition() {
        return position;
    }

    //XXX: rename to "calculateDeltaPose" because "get" has other meanings
    // seems OK-ish for now
    public pose getDeltaPose() {
        Average average = new Average();

        //average vertical translation
        double vertTransNet = average.ofAll(wheels, (Function<OdometryWheel, Double>) wheel ->
                wheel.distanceTraveledTowardsAngle(
                        wheel.getDeltaPosition(), facingForward));

        //average horizontal translation
        double horoTransNet = average.ofAll(wheels, (Function<OdometryWheel, Double>) wheel ->
                wheel.distanceTraveledTowardsAngle(
                        wheel.getDeltaPosition(), facingRight));

        //average rotation
        double rotAngNet = average.ofAll(wheels, (Consumer<OdometryWheel>) wheel ->
                average.add(
                        wheel.odoDeltaToBotAngle(
                                wheel.getDeltaPosition()
                                        - wheel.dotProduct(horoTransNet, facingRight)
                                        - wheel.dotProduct(vertTransNet, facingForward),
                                xCenterOfRotation, yCenterOfRotation),
                        //The farther, the heavier.  The more aligned to direction, the heavier
                        wheel.distanceToCenter() * Math.abs(wheel.dotProduct(
                                1,
                                wheel.ccTangentDir(xCenterOfRotation, yCenterOfRotation)))));

        return new pose(horoTransNet, vertTransNet, rotAngNet);
    }

    /**
     * Given the total forward translation, total sideways translation, and
     * total changed heading, and assuming that all three values change at a constant rate,
     * calculate the final change in position.
     * The calculus is tedious, but it works out nicely conceptually: imagine a line
     * from (x,y) to (x', y') and curve it into a sector of angle theta
     *
     * @return New curved pose
     */
    // XXX: In the jargon it's called a "twist".
    //  Maybe rename param to deltaPose
    //  Also I think this implementation is not 100% correct, but there are easy to find (more simple) implementations out there
    public pose curvedTrajectoryTranslation(pose in) {
        // if no rotation, then trig functions will be undefined
        if (in.r == 0) return in; //I think I spotted a bug; you forgot to rotate

        // define circle that contains the sector
        double arclength = Math.hypot(in.x, in.y);
        double radius = arclength / in.r;

        //curve the path around the sector
        point curved = new point(radius * Math.cos(in.r) - radius, radius * Math.sin(in.r));

        // maintain original direction of path
        point rotated = curved.rotate(-Math.PI / 2 + Math.atan2(in.y, in.x));

        return new pose(rotated.x, rotated.y, in.r);
    }
}
