package solution;

import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Rectangle2D;
import java.util.*;

public class ValidPoints {

    /** The workspace bounds */
    public static final Rectangle2D BOUNDS = new Rectangle2D.Double(0, 0, 1, 1);
    /** The default value for maximum error */
    public static final double DEFAULT_MAX_ERROR = 1e-5;

    ProblemSpec ps;

    PathPoint start;
    PathPoint goal;

    ArrayList<PathPoint> points;

    public ValidPoints (ProblemSpec ps) {

        this.ps = ps;

        ArrayList<PathPoint> points = new ArrayList<>();

        start = new PathPoint(ps.getInitialState().getASVPositions().get(0).getX(),
                ps.getInitialState().getASVPositions().get(0).getY());
        points.add(start);

        points.addAll(genAllPoints(ps.getObstacles(), ps.getASVCount()));

        goal = new PathPoint(ps.getGoalState().getASVPositions().get(0).getX(),
                ps.getGoalState().getASVPositions().get(0).getY());
        points.add(goal);

        this.points = genPointsWithEdges(points);
        traverse();

    }

    public Stack<PathPoint> getPath() {

        Stack<PathPoint> path = new Stack<>();

        path.add(goal);

        while (path.peek().getPreviousPointInPath() != null) {
            path.add(path.peek().getPreviousPointInPath());
        }
        return path;
    }

    /**
     * Traverse the points and update distance from start and previous point
     */
    public void traverse () {

        start.setDistanceFromStart(0);

        //Comparator and priority queue
        Comparator<PathPoint> comparator = new MinSrcDistComparator();
        PriorityQueue<PathPoint> queue = new PriorityQueue<>(comparator);

        queue.add(start);

        while (!queue.isEmpty()) {

            PathPoint currentPoint = queue.poll();

            currentPoint.setVisited();

            if (currentPoint.equals(goal)) {
                break;
            }

            ArrayList<Edge> edges = currentPoint.getEdges();

            for (Edge e : edges) {
                PathPoint nextPoint = e.getToPoint();
                double newDistance = currentPoint.getDistanceFromStart() + e.getLength();

                if (newDistance < nextPoint.getDistanceFromStart()) {
                    nextPoint.setDistanceFromStart(newDistance);
                    if (!nextPoint.isVisited())
                        nextPoint.setPreviousPointInPath(currentPoint);
                        queue.add(nextPoint);
                }
            }
        }
    }


    public PathPoint getStart() {
        return start;
    }

    public PathPoint getGoal() {
        return goal;
    }

    /**
     * Generate points with edges
     * @param validPoints
     * @return
     */
    public ArrayList<PathPoint> genPointsWithEdges (ArrayList<PathPoint> validPoints) {

        for (PathPoint p : validPoints) {
            for (PathPoint q : validPoints) {
                if (!collision(p, q) && !p.equals(q)) {
                    Edge edge = new Edge(p, q);
                    p.addEdge(edge);
                    q.addEdge(edge);
                }
            }
        }
        return validPoints;
    }

    /**
     * Check whether the line between 2 points intersect with any of the obstacles
     * @param p1
     * @param p2
     */
    public boolean collision(PathPoint p1, PathPoint p2) {
        Line2D line = new Line2D.Double(p1, p2);
        List<Obstacle> obstacles = ps.getObstacles();
        for (int i = 0; i<obstacles.size(); i++) {
            if (line.intersects(obstacles.get(i).getRect())) {
                return true;
            }
        }
        return false;
    }

    /**
     * Generate all points for given list of obstacles and ASV count
     * @param obstacles
     * @param ASVCount
     * @return
     */
    public ArrayList<PathPoint> genAllPoints(List<Obstacle> obstacles, int ASVCount) {
        ArrayList<PathPoint> points = new ArrayList<>();

        for (Obstacle o : obstacles) {
            points.addAll(validCornerPoints(o.getRect(), ASVCount));
        }

        return points;
    }


    /**
     * Generate valid corner points for a rectangle obstacle
     * @param rectangle
     * @param ASVCount
     * @return
     */
    public ArrayList<PathPoint> validCornerPoints(Rectangle2D rectangle, int ASVCount) {
        rectangle = grow(rectangle, getMinimumDiameter(ASVCount));

        ArrayList<PathPoint> points = new ArrayList<>();
        double[] arr = new double[2];

        for (PathIterator pi = rectangle.getPathIterator(null); !pi.isDone(); pi.next()) {
            if (pi.currentSegment(arr) == PathIterator.SEG_LINETO) {
                PathPoint point = new PathPoint(arr[0], arr[1]);
                if (grow(BOUNDS, DEFAULT_MAX_ERROR).contains(point)) {
                    points.add(point);
                }
            }
        }

        return points;
    }

    /**
     * Returns the minimum area required for the given number of ASVs.
     *
     * @return the minimum area required.
     */
    public final double getMinimumDiameter(int ASVCount) {
        double radius = 0.007 * (ASVCount - 1);
        return radius/2;
    }

    /**
     * Creates a new Rectangle2D that is grown by delta in each direction
     * compared to the given Rectangle2D.
     *
     * @param rect
     *            the Rectangle2D to expand.
     * @param delta
     *            the amount to expand by.
     * @return a Rectangle2D expanded by delta in each direction.
     */
    public Rectangle2D grow(Rectangle2D rect, double delta) {
        return new Rectangle2D.Double(rect.getX() - (delta*4), rect.getY() - delta,
                rect.getWidth() + delta * 8, rect.getHeight() + delta * 2);
    }

}
