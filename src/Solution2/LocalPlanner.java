package Solution2;

import problem.ASVConfig;
import problem.ProblemSpec;
import solution.PathPoint;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

/**
 * If a collision free path exist from one config to another, give all path points with minimum step distance.
 */
public class LocalPlanner {

    ASVConfig startConfig;
    ASVConfig nextConfig;
    ProblemSpec ps;
    Tester tester;

    public LocalPlanner(ASVConfig startConfig, ASVConfig nextConfig, ProblemSpec ps, Tester tester) {
        this.startConfig = startConfig;
        this.nextConfig = nextConfig;
        this.ps = ps;
        this.tester = tester;
    }

    public LocalPlanner() {

    }

    /**
     * Check if collision free path exist from one config to other
     * @return
     */
    public boolean freePathExist(ASVConfig startConfig, ASVConfig nextConfig) {
        for (int i = 0; i < startConfig.getASVPositions().size(); i++) {
            Line2D line = new Line2D.Double(startConfig.getASVPositions().get(i), nextConfig.getASVPositions().get(i));
            for (int j = 0; j < ps.getObstacles().size(); j++) {
                if (line.intersects(ps.getObstacles().get(j).getRect())) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Get collision free path between start and end config
     * @return
     */
    public List<ASVConfig> getFreePath() {
        List<ASVConfig> path = new ArrayList<>();

        ASVConfig current = startConfig;
        ASVConfig next = nextConfig;

        while (!sameConfig(current, next)) {
            ASVConfig nextStep = nextStep(current, next);
            if (tester.validConfig(nextStep, ps)) {
                path.add(nextStep);
                current = nextStep;
            }else {
                return path;
            }
        }

        return path;
    }

    /**
     * Returns next step of ASV configs on a line with a step 0.001
     * @param currentConfig
     * @return
     */
    public ASVConfig nextStep(ASVConfig currentConfig, ASVConfig next) {

        List<Point2D> nextStep = new ArrayList<>();
        for (int i = 0; i< currentConfig.getASVPositions().size(); i++) {
            nextStep.add(nextPoint(currentConfig.getASVPositions().get(i), next.getASVPositions().get(i), Tester.MAX_STEP));
        }
        return new ASVConfig(nextStep);
    }

    /**
     * Returns next point on the line by the interval 0.001
     * @param p1
     * @param p2
     * @return
     */
    private Point2D nextPoint(Point2D p1, Point2D p2, double stepSize) {
        double diffX = p2.getX() - p1.getX();
        double diffY = p2.getY() - p1.getY();
        double numberOfPoints = p1.distance(p2);

        double intervalX = diffX / (numberOfPoints);
        double intervalY = diffY / (numberOfPoints);

        Point2D nextPoint = p2;
        if (numberOfPoints <= stepSize) {
            return nextPoint;
        }

        double x = p1.getX()+intervalX* stepSize;
        double y = p1.getY()+intervalY* stepSize;

        return new PathPoint(x,y);
    }

    /**
     * Return the list of configs until the start config is expanded to the maximum
     * @return
     */
    public List<ASVConfig> getMaximumExpansion() {
        List<ASVConfig> expanded = new ArrayList<>();

        ASVConfig current = startConfig;
        while (tester.validConfig(current, ps)) {
            List<ASVConfig> ex = expand(current, Tester.DEFAULT_EXP_ANGLE*startConfig.getConvexityDirection());
            if (tester.validConfig(ex.get(0), ps) && tester.validConfig(ex.get(ex.size()-1), ps)) {
                expanded.addAll(ex);
                current = ex.get(ex.size()-1);
            }else {
                current = ex.get(ex.size()-1);
            }
        }

        return expanded;
    }

    /**
     * Moving a config with respect to direction given
     * @param configToMove
     * @param up
     * @param down
     * @param right
     * @param left
     * @return
     */
    private ASVConfig move(ASVConfig configToMove, double up, double down, double right, double left) {
        List<Point2D> result = configToMove.getASVPositions();
        for (int i = 0; i < result.size(); i++) {
            result.set(i, new Point2D.Double(result.get(i).getX() - left + right,
                    result.get(i).getY() - down + up));
        }
        return new ASVConfig(result);
    }

    /**
     * DO the actual expansion
     * @param toExpand
     * @return
     */
    private List<ASVConfig> expand(ASVConfig toExpand, double angle) {
        List<ASVConfig> expanded = new ArrayList<>();

        List<Point2D> positions = toExpand.getASVPositions();

        for (int i = 0; i < positions.size()-1; i++) {
            for (int j = i + 1; j < positions.size(); j++) {
                positions.set(j, rotatePoints(positions.get(j), positions.get(i), angle));
            }
            expanded.add(new ASVConfig(positions));
        }
        return expanded;
    }

    /**
     * Rotate ASVConfig one degree
     * @param toRorate
     * @return
     */
    protected ASVConfig rotate(ASVConfig toRorate, double angle) {
        List<Point2D> rotated = toRorate.getASVPositions();
        for (int i=0; i<rotated.size()-1; i++) {
            rotated.set(i+1, rotatePoints(rotated.get(i+1), rotated.get(0), angle));
        }

        return new ASVConfig(rotated);
    }

    /**
     * Rotate a point with specified angle and specified center
     * @param toRotate
     * @param center
     * @param angle
     * @return
     */
    private Point2D rotatePoints(Point2D toRotate, Point2D center, double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = ((toRotate.getX() - center.getX())*cos - (toRotate.getY() - center.getY())*sin) + center.getX();
        double y = ((toRotate.getX() - center.getX())*sin + (toRotate.getY() - center.getY())*cos) + center.getY();

        return new Point2D.Double(x,y);
    }

    /**
     * Generate all points on a line
     * @param p1
     * @param p2
     */
    public ArrayList<Point2D> allPointsOnPath(Point2D p1, Point2D p2) {

        double diffX = p2.getX() - p1.getX();
        double diffY = p2.getY() - p1.getY();
        double numberOfPoints = p1.distance(p2);

        double intervalX = diffX / (numberOfPoints);
        double intervalY = diffY / (numberOfPoints);

        ArrayList<Point2D> points = new ArrayList<>();
        if (numberOfPoints == 0) {
            return points;
        }

        for (double i = 0; i < numberOfPoints+(Tester.MAX_STEP/2); i=i+(Tester.MAX_STEP/2)) {
            double x = p1.getX()+intervalX*i;
            double y = p1.getY()+intervalY*i;
            points.add(new Point2D.Double(x,y));
        }
        return points;
    }

    /**
     * Returns if a config is same as another
     * @param cfg1
     * @param cfg2
     * @return
     */
    protected boolean sameConfig(ASVConfig cfg1, ASVConfig cfg2) {
        return cfg1.maxDistance(cfg2) <= Tester.MAX_STEP;
    }


}
