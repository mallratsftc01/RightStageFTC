package epra.math.geometry;

/**Stores an length, theta vector.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Vector extends Angle {

    private double length;

    /**Stores an length, theta vector.
     *<p></p>
     *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * @param length
     * @param theta*/
    public Vector(double length, Angle theta) {
        super(theta.getDegree());
        this.length = length;
    }

    /**Stores an length, theta vector.
     *<p></p>
     *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * @param x
     * @param y*/
    public Vector(double x, double y) {
        super((float) ((y < 0.0) ? Math.PI - Math.atan(x / y) : Math.atan(x / y)));
        this.length = Geometry.pythagorean(x, y);
    }

    /**Stores an length, theta vector.
     *<p></p>
     *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * @param point Point at the end of the vector.*/
    public Vector(Point point) {
        super((float) ((point.y < 0.0) ? Math.PI - Math.atan(point.x / point.y) : Math.atan(point.x / point.y)));
        this.length = Geometry.pythagorean(point.x, point.y);
    }

    /**@return The length of the vector.*/
    public double getLength() { return length; }
    /**@param length The new length of the vector.*/
    public void setLength(double length) { this.length = length; }

    /**@param point Point at the end of the vector.*/
    public void setPoint(Point point) {
        double radian = Math.atan(point.x / point.y);
        if (point.y < 0.0) { radian = Math.PI - radian; }
        super.setRadian(radian);
        this.length = Geometry.pythagorean(point.x, point.y);
    }

    /**@return The vector converted to an x, y coordinate point/*/
    public Point toPoint() { return new Point(Math.cos(super.getRadian()) * length, Math.sin(super.getRadian()) * length); }
    /**@return A right triangle between 0,0, vector x,0, and vector x, vector y.*/
    public Triangle toTriangle() { return new Triangle(this); }
}
