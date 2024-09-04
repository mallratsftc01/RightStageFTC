package epra.math.geometry;

/**A class that adds many geometric functions for many uses.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Geometry {
    /**A unit vector pointing to the right.*/
    public static final Vector I_HAT = new Vector(1, new Angle(0.0));
    /**A unit vector pointing upwards.*/
    public static final Vector J_HAT = new Vector(1, new Angle(90.0));

    public Geometry() {}

    /**@param angle1 First angle.
     * @param angle2 Second angle.
     * @return The resulting angle from adding the first to the second, in a range between 0 and 360 degrees (0 to 2pi).*/
    public static Angle add(Angle angle1, Angle angle2) { return new Angle((angle1.getDegree() + angle2.getDegree()) % 360); }
    /**@param angle1 First angle.
     * @param angle2 Second angle.
     * @return The resulting angle from subtracting the second from the first, in a range between 0 and 360 degrees (0 to 2pi).*/
    public static Angle subtract(Angle angle1, Angle angle2) { return new Angle((angle1.getDegree() - angle2.getDegree()) % 360); }
    /**@param angle Array of angles.
     * @return The average angle of the array.*/
    public static Angle average(Angle[] angle) {
        double degrees = 0.0;
        for (Angle a : angle) { degrees += a.getDegree(); }
        return new Angle(degrees % 360.0);
    }

    /**@param point1 First point.
     * @param point2 Second point.
     * @return The resulting point from adding the first to the second.*/
    public static Point add(Point point1, Point point2) { return new Point(point1.x + point2.x, point1.y + point2.y); }
    /**@param point1 First point.
     * @param point2 Second point.
     * @return The resulting point from subtracting the second from the first.*/
    public static Point subtract(Point point1, Point point2) { return new Point(point1.x - point2.x, point1.y - point2.y); }
    /**@param point Array of points.
     * @return The average point of the array.*/
    public static Point average(Point[] point) {
        double x = 0.0;
        double y = 0.0;
        for (Point p : point) {
            x += p.x;
            y += p.y;
        }
        return new Point(x, y);
    }

    /**@param vector1 First vector.
     * @param vector2 Second vector.
     * @return The resulting point from adding the first to the second.*/
    public static Vector add(Vector vector1, Vector vector2) { return new Vector(add(vector1.toPoint(), vector2.toPoint())); }
    /**@param vector1 First vector.
     * @param vector2 Second vector.
     * @return The resulting point from subtracting the second from the first.*/
    public static Vector subtract(Vector vector1, Vector vector2) { return new Vector(subtract(vector1.toPoint(), vector2.toPoint())); }
    /**@param vector1 First vector.
     * @param vector2 Second vector.
     * @return Returns the dot product of the two vectors.*/
    public static double dot(Vector vector1, Vector vector2) { return (vector1.toPoint().x * vector2.toPoint().x) + (vector1.toPoint().y * vector2.toPoint().y); }

    /**@param a Point a.
     * @param b Point b.
     * @return The distance between point a and b.*/
    public static double pythagorean(Point a, Point b) { return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2)); }
    /**@param a Length of leg a.
     * @param b Length of leg b.
     * @return The length of the hypotenuse of a triangle with legs a and b.*/
    public static double pythagorean(double a, double b) { return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2)); }

    /**@param angle1 First angle.
     * @param angle2 Second angle.
     * @return The direction for the first angle to reach the second angle the quickest (1.0 is clockwise, -1.0 is counterclockwise).*/
    public static double direction(Angle angle1, Angle angle2) {
        double sign = Math.signum(angle1.getDegree() - angle2.getDegree());
        if (Math.max(angle1.getDegree() % 360, angle2.getDegree() % 360) > 270 && Math.min(angle1.getDegree() % 360, angle2.getDegree() % 360) < 90) {
            sign *= -1;
        }
        return sign;
    }
}