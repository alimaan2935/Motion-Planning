package solution;

import java.awt.geom.Line2D;

public class Edge extends Line2D.Double {

    PathPoint fromPoint;
    PathPoint toPoint;

    double length = 0;

    public Edge (PathPoint from, PathPoint to) {
        super(from, to);
        fromPoint = from;
        toPoint = to;
    }

    public PathPoint getFromPoint() {
        return fromPoint;
    }

    public void setFromPoint(PathPoint fromPoint) {
        this.fromPoint = fromPoint;
        super.x1 = fromPoint.getX();
        super.y1 = fromPoint.getY();
    }

    public PathPoint getToPoint() {
        return toPoint;
    }

    public void setToPoint(PathPoint toPoint) {
        this.toPoint = toPoint;
        super.x2 = toPoint.getX();
        super.y2 = toPoint.getY();
    }

    public double getLength() {
        length = fromPoint.distance(toPoint);
        return length;
    }
}
