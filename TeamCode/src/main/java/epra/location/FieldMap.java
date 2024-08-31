package epra.location;

import epra.math.geometry.Point;
import epra.math.geometry.Shape2D;

import java.util.HashMap;
import java.util.Map;

/**A basic interface to build field maps off of.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public interface FieldMap {

    double length = 0.0;
    double width = 0.0;

    /**Holds all the different sections of the field.*/
    enum Section {
        DOES_NOT_EXIST,
        OBSTACLE
    }

    /**Map of all sections and associated component.*/
    Map<Section, Shape2D> map = new HashMap<>();

    /**@param point Point to check.
     * @return Which section of the field the point falls into. Returns DOES_NOT_EXIST if the point doesn't fall into any section of the field.*/
    default Section checkPoint(Point point) {
        for (Map.Entry<Section, Shape2D> entry : map.entrySet()) {
            if (entry.getValue().checkPoint(point)) { return entry.getKey(); }
        }
        return Section.DOES_NOT_EXIST;
    }
}
