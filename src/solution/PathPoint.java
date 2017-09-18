package solution;

import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

public class PathPoint extends Point2D.Double {

    Ellipse2D pointCircle;

    double distanceFromStart = Integer.MAX_VALUE;
    boolean isVisited = false;

    PathPoint previousPointInPath;

    ArrayList<Edge> edges = new ArrayList<>();

    public PathPoint (double x, double y) {
        super(x, y);
    }

    public double getDistanceFromStart() {
        return distanceFromStart;
    }

    public void setDistanceFromStart(double distanceFromStart) {
        this.distanceFromStart = distanceFromStart;
    }

    public Ellipse2D getPointCircle() {
        return pointCircle;
    }

    public void setPointCircle(double radius) {
        this.pointCircle = getEllipseFromCenter(super.getX(), super.getY(), radius, radius);
    }

    public boolean isVisited() {
        return isVisited;
    }

    public void setVisited() {
        isVisited = true;
    }

    public void addEdge(Edge edge) {
        edges.add(edge);
    }

    public ArrayList<Edge> getEdges() {
        return edges;
    }

    public PathPoint getPreviousPointInPath() {
        return previousPointInPath;
    }

    public void setPreviousPointInPath(PathPoint previousPointInPath) {
        this.previousPointInPath = previousPointInPath;
    }

    private Ellipse2D getEllipseFromCenter(double x, double y, double width, double height)
    {
        double newX = x - width / 2.0;
        double newY = y - height / 2.0;

        Ellipse2D ellipse = new Ellipse2D.Double(newX, newY, width, height);

        return ellipse;
    }
}
