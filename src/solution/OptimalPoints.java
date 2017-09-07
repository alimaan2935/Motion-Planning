package solution;

import javafx.util.Pair;
import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class OptimalPoints {

    ArrayList<Point2D> allPoints = new ArrayList<>();
    ProblemSpec ps;

    public OptimalPoints(ProblemSpec ps) {
        this.ps = ps;
    }

    /**
     *
     */
    public ArrayList<Point2D> getPoints() {

        Point2D p1 = ps.getInitialState().getASVPositions().get(0);
        Point2D p2 = ps.getGoalState().getASVPositions().get(0);
        getAllPoints(p1, p2);
        return allPoints;

    }


    /******************************************************************************************************************/
    /*******************  Step 1: See if Start and goal point intersects with an obstacle - return the obstacle   ***********/
    /******************************************************************************************************************/

    /**
     * Check whether the line between 2 points intersect with any of the obstacles
     * @param p1
     * @param p2
     */
    public Obstacle lineIntersect(Point2D p1, Point2D p2) {
        Line2D line = new Line2D.Double(p1, p2);
        List<Obstacle> obstacles = ps.getObstacles();
        for (int i = 0; i<obstacles.size(); i++) {
            if (line.intersects(obstacles.get(i).getRect())) {
                return obstacles.get(i);
            }
        }
        return null;
    }

    /******************************************************************************************************************/
    /*******************  Step 2: Generate all points on a line - returns ArrayList of Point2D   ***********/
    /******************************************************************************************************************/

    /**
     * Generate all points in a line
     * @param p1
     * @param p2
     */
    public ArrayList<Point2D> allPointsOnLine(Point2D p1, Point2D p2) {

        double diffX = p2.getX() - p1.getX();
        double diffY = p2.getY() - p1.getY();
        double numberOfPoints = p1.distance(p2);

        double intervalX = diffX / (numberOfPoints);
        double intervalY = diffY / (numberOfPoints);

        ArrayList<Point2D> points = new ArrayList<>();

        DecimalFormat numberFormat = new DecimalFormat("0.000");

        for (double i = 0; i < numberOfPoints+0.001; i=i+0.001) {
            double x = Double.parseDouble(numberFormat.format(p1.getX()+intervalX*i));
            double y = Double.parseDouble(numberFormat.format(p1.getY()+intervalY*i));
            points.add(new Point2D.Double(x,y));
        }

        return points;

    }


    /******************************************************************************************************************/
    /*******************  Step 3: If a line intersects with an obstacle, determine the lines of intersection   ***********/
    /******************************************************************************************************************/



    public Line2D[] getIntersectionPoints(Rectangle2D rectangle, Line2D line) {

        Line2D[] lines = new Line2D[4];
        Line2D topLine = new Line2D.Double(
                rectangle.getX(),
                rectangle.getY(),
                rectangle.getX() + rectangle.getWidth(),
                rectangle.getY());
        Line2D bottomLine = new Line2D.Double(
                rectangle.getX(),
                rectangle.getY() + rectangle.getHeight(),
                rectangle.getX() + rectangle.getWidth(),
                rectangle.getY() + rectangle.getHeight());
        Line2D leftLine = new Line2D.Double(
                rectangle.getX(),
                rectangle.getY(),
                rectangle.getX(),
                rectangle.getY() + rectangle.getHeight());
        Line2D rightLine = new Line2D.Double(
                rectangle.getX() + rectangle.getWidth(),
                rectangle.getY(),
                rectangle.getX() + rectangle.getWidth(),
                rectangle.getY() + rectangle.getHeight());

        // Top line
        if (getIntersectionPoint(line, topLine) != null) {
            lines[0] = topLine;
        }
        // Bottom line
        if (getIntersectionPoint(line, bottomLine) != null) {
            lines[1] = bottomLine;
        }
        // Left side...
        if (getIntersectionPoint(line, leftLine) != null) {
            lines[2] = leftLine;
        }
        // Right side
        if (getIntersectionPoint(line, rightLine) != null) {
            lines[3] = rightLine;
        }

        return lines;

    }

    public Point2D getIntersectionPoint(Line2D lineA, Line2D lineB) {

        double x1 = lineA.getX1();
        double y1 = lineA.getY1();
        double x2 = lineA.getX2();
        double y2 = lineA.getY2();

        double x3 = lineB.getX1();
        double y3 = lineB.getY1();
        double x4 = lineB.getX2();
        double y4 = lineB.getY2();

        Point2D p = null;

        double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (d != 0) {
            double xi = ((x3 - x4) * (x1 * y2 - y1 * x2) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;
            double yi = ((y3 - y4) * (x1 * y2 - y1 * x2) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;

            p = new Point2D.Double(xi, yi);

        }
        return p;
    }


    /******************************************************************************************************************/
    /*******************  Step 4: Determine which line of obstacle a particular intersecting point lies   ***********/
    /******************************************************************************************************************/

    /**
     * Returns the line of the obstacle where point is intersecting
     * @param point
     */
    public Line2D whichLine(Point2D point, Line2D[] lines) {

        double distance = 1;
        Line2D line = new Line2D.Double();

        for (int i=0; i<lines.length; i++) {
            if (lines[i] != null) {
                if (lines[i].ptLineDist(point) < distance) {
                    distance = lines[i].ptLineDist(point);
                    line = lines[i];
                }
            }
        }
        return line;
    }

    /******************************************************************************************************************/
    /*******************  Step 5: Generate two alternate points based on the intersecting line   ***********/
    /******************************************************************************************************************/


    public Pair<Point2D, Point2D> pointsPair (Line2D line) {
        Point2D p1 = line.getP1();
        Point2D p2 = line.getP2();
        double dist = p1.distance(p2);

        p1 = new Point2D.Double(p1.getX()-0.036, p1.getY()-0.036);
        p2 = new Point2D.Double(p2.getX()+0.036, p2.getY()+0.036);

        return new Pair<>(p1, p2);
    }


    /******************************************************************************************************************/
    /*******************  Step 6: Forms a line between newly generated point and check the intersection - repeat the process   ***********/
    /******************************************************************************************************************/




    /******************************************************************************************************************/
    /*******************  Step 7: Populate the list of alternate points   ***********/
    /******************************************************************************************************************/


    /**
     * Generate all points from initial to goal without colliding with obstacles
     * @return
     */
    public void getAllPoints(Point2D p1, Point2D p2) {

        allPoints.add(p1);
        Rectangle2D intersection;
        Line2D[] lines = new Line2D[2];

        if (lineIntersect(p1, p2) == null) {
            allPoints.add(p2);
            return;
        }else {
            intersection = lineIntersect(p1, p2).getRect();
            try {
                lines = getIntersectionPoints(intersection, new Line2D.Double(p1, p2));

            }catch (Exception e){
                e.printStackTrace();
            }
        }

        Line2D line = whichLine(p1, lines);
        Pair<Point2D, Point2D> pointPair = pointsPair(line);
        Point2D desiredPoint = whichPoint(pointPair);
        allPoints.add(desiredPoint);

        getAllPoints(desiredPoint, p2);

    }

    /**
     * Which point to choose from a pair of points
     * based on distance to goal and within bounding box
     * @param pair
     * @return
     */
    public Point2D whichPoint(Pair<Point2D, Point2D> pair) {

        Rectangle2D rect = new Rectangle2D.Double(0, 0, 1, 1);

        double distance = Integer.MAX_VALUE;

        if (rect.contains(pair.getKey())) {
            distance = ps.getGoalState().getASVPositions().get(0).distance(pair.getKey());
        }

        if (rect.contains(pair.getValue())) {
            double newDistance = ps.getGoalState().getASVPositions().get(0).distance(pair.getValue());
            if (newDistance < distance) {
                return pair.getValue();
            }else {
                return pair.getKey();
            }
        }
        return pair.getKey();

    }


    /******************************************************************************************************************/
    /*******************  Step 8: Get all points joining all lines and generate ASV configs   ***********/
    /******************************************************************************************************************/




    /**
     * Generate All for initial and goal ASV configs
     * @param initial
     * @param goal
     */
    public List<ASVConfig> genASVConfigs(ASVConfig initial, ASVConfig goal) {

        List<ASVConfig> path = new ArrayList<>();

        List<ArrayList<Point2D>> configs = new ArrayList<>();

        List<Point2D> initialPositions = initial.getASVPositions();
        List<Point2D> goalPositions = goal.getASVPositions();

        for (int i = 0; i< initial.getASVCount(); i++) {
            Point2D p1 = initialPositions.get(i);
            Point2D p2 = goalPositions.get(i);
            configs.add(allPointsOnLine(p1, p2));
        }

        for (int i = 0; i<configs.get(0).size(); i++) {
            double[] config = new double[configs.size()*2];
            for (int j = 0; j<configs.size(); j++) {
                config[j*2] = configs.get(j).get(i).getX();
                config[j*2+1] = configs.get(j).get(i).getY();
            }

            path.add(new ASVConfig(config));
        }
        return path;
    }


}
