package solution;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class OptimalConfigs {

    ArrayList <ASVConfig> optimalConfigs = new ArrayList<>();

    Line2D line = new Line2D.Double();

    public double goalDistance(ProblemSpec ps) {


        List<ASVConfig> path = genASVConfigs(ps.getInitialState(), ps.getGoalState());

        ps.setPath(path);

        try {
            ps.saveSolution("testcases/exp.txt");
        }catch (IOException e) {

        }



        List<Obstacle> o = ps.getObstacles();

        List<Point2D> initial = ps.getInitialState().getASVPositions();
        List<Point2D> goal = ps.getGoalState().getASVPositions();

        allPointsOnLine(initial.get(0), goal.get(0));

        for (int i = 0; i<initial.size(); i++) {
            double distance  = initial.get(i).distance(goal.get(i));

            line.setLine(initial.get(i), goal.get(i));

            System.out.println(line.intersects(o.get(0).getRect()));
            System.out.println(distance);
        }
        return initial.get(0).getX();
    }

    /**
     * Check whether the line between 2 points intersect with any of the obstacles
     * @param p1
     * @param p2
     */
    public boolean lineIntersect(Point2D p1, Point2D p2) {
        return true;
    }

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
            double y = Double.parseDouble(numberFormat.format(p2.getX()+intervalY*i));
            points.add(new Point2D.Double(x,y));
        }

        for (int i = 0; i<points.size(); i++) {
            System.out.println(points.get(i).getX() + "    " + points.get(i).getY());
        }

        return points;

    }

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
            double[] config = {configs.get(0).get(i).getX(),
                                configs.get(0).get(i).getY(),
                                configs.get(1).get(i).getX(),
                                configs.get(1).get(i).getY()};
            path.add(new ASVConfig(config));
        }
        return path;
    }

    /**
     * Generate alternate point if the line intersects any of the obstacles
     */
    public void genAlterPoint() {

    }

}
