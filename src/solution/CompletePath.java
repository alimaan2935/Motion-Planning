package solution;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Stack;

public class CompletePath {

    private ProblemSpec ps;
    private Stack<PathPoint> path;

    private static final double DEFAULT_MAX_ERROR = 1e-5;


    /**
     * Constructor
     * @param ps
     * @param path
     */
    public CompletePath (ProblemSpec ps, Stack<PathPoint> path) {
        this.ps = ps;
        this.path = path;
    }

    /**
     * @return complete path as list of asv configs
     */
    public List<ASVConfig> getCompletePath() {
        return completePath(ps.getInitialState(), ps.getGoalState(), path);
    }


    /**
     * Main driver method of the algorithm
     * @param startConfig
     * @param goalConfig
     * @param path
     * @return
     */
    private List<ASVConfig> completePath(ASVConfig startConfig, ASVConfig goalConfig, Stack<PathPoint> path) {
        List<ASVConfig> completePath = new ArrayList<>();
        completePath.add(startConfig);
        path.pop();

        while (path.size() > 0) {
            Point2D nextPoint = path.pop();
            ASVConfig current = completePath.get(completePath.size()-1);
            completePath.addAll(transformation(current, nextPoint));
        }

        completePath.add(goalConfig); // testing
        return completePath;
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
    private Rectangle2D grow(Rectangle2D rect, double delta) {
        return new Rectangle2D.Double(rect.getX() - (delta*4), rect.getY() - delta,
                rect.getWidth() + delta * 4, rect.getHeight() + delta * 2);
    }

    private boolean hasEnoughArea(ASVConfig cfg) {
        double total = 0;
        List<Point2D> points = cfg.getASVPositions();
        points.add(points.get(0));
        points.add(points.get(1));
        for (int i = 1; i < points.size() - 1; i++) {
            total += points.get(i).getX()
                    * (points.get(i + 1).getY() - points.get(i - 1).getY());
        }
        double area = Math.abs(total) / 2;
        double minArea = getMinimumArea(cfg.getASVCount()) - 1e-5;
        return (area >= minArea);
    }


    /**
     * Normalises an angle to the range (-pi, pi]
     *
     * @param angle
     *            the angle to normalise.
     * @return the normalised angle.
     */
    public double normaliseAngle(double angle) {
        while (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Returns whether the given configuration is convex.
     *
     * @param cfg
     *            the configuration to test.
     * @return whether the given configuration is convex.
     */
    public boolean isConvex(ASVConfig cfg) {
        List<Point2D> points = cfg.getASVPositions();
        points.add(points.get(0));
        points.add(points.get(1));

        double requiredSign = 0;
        double totalTurned = 0;
        Point2D p0 = points.get(0);
        Point2D p1 = points.get(1);
        double angle = Math.atan2(p1.getY() - p0.getY(), p1.getX() - p0.getX());
        for (int i = 2; i < points.size(); i++) {
            Point2D p2 = points.get(i);
            double nextAngle = Math.atan2(p2.getY() - p1.getY(),
                    p2.getX() - p1.getX());
            double turningAngle = normaliseAngle(nextAngle - angle);

            if (turningAngle == Math.PI) {
                return false;
            }

            totalTurned += Math.abs(turningAngle);
            if (totalTurned > 3 * Math.PI) {
                return false;
            }

            double turnSign;
            if (turningAngle < -DEFAULT_MAX_ERROR) {
                turnSign = -1;
            } else if (turningAngle > DEFAULT_MAX_ERROR) {
                turnSign = 1;
            } else {
                turnSign = 0;
            }

            if (turnSign * requiredSign < 0) {
                return false;
            } else if (turnSign != 0) {
                requiredSign = turnSign;
            }

            p0 = p1;
            p1 = p2;
            angle = nextAngle;
        }
        return true;
    }

    private final double getMinimumArea(int asvCount) {
        double radius = 0.007 * (asvCount - 1);
        return Math.PI * radius * radius;
    }

    private boolean hasValidBoomLengths(ASVConfig cfg) {
        List<Point2D> points = cfg.getASVPositions();
        for (int i = 1; i < points.size(); i++) {
            Point2D p0 = points.get(i - 1);
            Point2D p1 = points.get(i);
            double boomLength = p0.distance(p1);
            if (boomLength < 0.05 - 1e-5) {
                return false;
            } else if (boomLength > 0.05 + 1e-5) {
                return false;
            }
        }
        return true;
    }
    private boolean isValidStep(ASVConfig cfg0, ASVConfig cfg1) {
        return (cfg0.maxDistance(cfg1) <= 0.001 + 1e-5);
    }


    private List<ASVConfig> transformation (ASVConfig current, Point2D nextPoint) {
        List<ASVConfig> transformedList = new ArrayList<>();

        ASVConfig c = current;

        while (hasEnoughArea(c) && isConvex(c)) {
            List<Point2D> positions = c.getASVPositions();

            if (hasCollision(c) != -1) {
                positions = changePointList(positions);
                c = new ASVConfig(positions);
                transformedList.add(c);
            }else {
                for (int i = 1; i < positions.size() - 1; i++) {
                    for (int j = i + 1; j < positions.size(); j++) {
                        positions.set(j, rotatePoints(positions.get(j), positions.get(i), -1));
                    }
                    c = new ASVConfig(positions);
                    transformedList.add(c);
                }
            }
        }

        ASVConfig nextPointConfig = nextConfig(c, nextPoint);
        ASVConfig nextStep = nextStep(c, nextPointConfig);

        int test = 1000;

        while (!samePoint(nextStep.getASVPositions().get(0), nextPoint) && test>1) {

            if (hasCollision(nextStep) != -1) {
                nextStep = rotateAroundFirst(nextStep);
                transformedList.add(nextStep);
            }else {
                nextStep = nextStep(nextStep, nextPointConfig);
                transformedList.add(nextStep);
            }
            test--;
        }




        return transformedList;
    }

    private List<ASVConfig> expandConfig(ASVConfig toExpand) {
        List<ASVConfig> expanded = new ArrayList<>();

        List<Point2D> positions = toExpand.getASVPositions();

        for (int i = 1; i < positions.size() - 1; i++) {
            for (int j = i + 1; j < positions.size(); j++) {
                positions.set(j, rotatePoints(positions.get(j), positions.get(i), -1));
            }
            expanded.add(new ASVConfig(positions));
        }
        return expanded;
    }

    private ASVConfig rotateAroundFirst(ASVConfig toRorate) {
        List<Point2D> rotated = toRorate.getASVPositions();
        for (int i=0; i<rotated.size()-1; i++) {
            rotated.set(i+1, rotatePoints(rotated.get(i+1), rotated.get(0), -1));
        }
        return new ASVConfig(rotated);
    }

    private boolean samePoint(Point2D p1, Point2D p2) {
        return p1.distance(p2) <= DEFAULT_MAX_ERROR;
    }

    /**
     * Returns a copy of Point2D list where each x is changed by minimum step.
     *
     * @param list
     *            the list of Point2D to change to.
     * @return a copy of list where each x is changed by minimum step.
     */
    public List<Point2D> changePointList(List<Point2D> list) {
        List<Point2D> result = new ArrayList<Point2D>();
        for (Point2D p : list) {
            result.add(new Point2D.Double(p.getX()-0.001, p.getY()));
        }
        return result;
    }

    private Point2D rotatePoints(Point2D toRotate, Point2D center, double angle) {
        angle = angle * Math.PI/180;
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = ((toRotate.getX() - center.getX())*cos - (toRotate.getY() - center.getY())*sin) + center.getX();
        double y = ((toRotate.getX() - center.getX())*sin + (toRotate.getY() - center.getY())*cos) + center.getY();

        return new Point2D.Double(x,y);
    }

    private ASVConfig nextConfig(ASVConfig current, Point2D point) {
        List<Point2D> currentConfigs = current.getASVPositions();
        List<Point2D> transformed = new ArrayList<>(currentConfigs.size());
        transformed.add(point);

        for (int i = 0; i < currentConfigs.size()-1; i++) {
            Point2D p0 = currentConfigs.get(i);
            Point2D p1 = currentConfigs.get(i+1);

            double distance = 0.05;

            double angle = Math.atan2(p1.getY() - p0.getY(), p1.getX() - p0.getX());
            double x = point.getX() + (Math.cos(angle)*distance);
            double y = point.getY() + (Math.sin(angle)*distance);
            PathPoint thisPoint = new PathPoint(x,y);
            point = thisPoint;
            transformed.add(thisPoint);
        }

        return new ASVConfig(transformed);

    }

    private Point2D nextPoint(Point2D p1, Point2D p2) {
        double diffX = p2.getX() - p1.getX();
        double diffY = p2.getY() - p1.getY();
        double numberOfPoints = p1.distance(p2);

        double intervalX = diffX / (numberOfPoints);
        double intervalY = diffY / (numberOfPoints);

        Point2D nextPoint = p1;
        if (numberOfPoints == 0) {
            return nextPoint;
        }
        DecimalFormat numberFormat = new DecimalFormat("0.000");

        double x = Double.parseDouble(numberFormat.format(p1.getX()+intervalX*0.001));
        double y = Double.parseDouble(numberFormat.format(p1.getY()+intervalY*0.001));

        return new PathPoint(x,y);
    }

    private ASVConfig nextStep(ASVConfig currentConfig, ASVConfig nextConfig) {

        List<Point2D> nextStep = new ArrayList<>();
        for (int i = 0; i< currentConfig.getASVPositions().size(); i++) {
            nextStep.add(nextPoint(currentConfig.getASVPositions().get(i), nextConfig.getASVPositions().get(i)));
        }
        return new ASVConfig(nextStep);
    }

    /**
     * Returns whether the given config collides with any of the given
     * obstacles.
     *
     * @param cfg
     *            the configuration to test.
     * @return whether the given config collides with any of the given
     *         obstacles.
     */
    private int hasCollision(ASVConfig cfg) {
        List<Obstacle> obstacles = ps.getObstacles();
        for (Obstacle o : obstacles) {
            int i = hasCollision(cfg, o);
            if (i != -1) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Returns whether the given config collides with the given obstacle.
     *
     * @param cfg
     *            the configuration to test.
     * @param o
     *            the obstacle to test against.
     * @return whether the given config collides with the given obstacle.
     */
    private int hasCollision(ASVConfig cfg, Obstacle o) {
        Rectangle2D lenientRect = grow(o.getRect(), -DEFAULT_MAX_ERROR);
        List<Point2D> points = cfg.getASVPositions();
        for (int i = 1; i < points.size(); i++) {
            if (new Line2D.Double(points.get(i - 1), points.get(i))
                    .intersects(lenientRect)) {
                return i;
            }
        }
        return -1;
    }

    private boolean sameConfig(ASVConfig cfg1, ASVConfig cfg2) {
        return cfg1.maxDistance(cfg2) <= DEFAULT_MAX_ERROR;
    }


}
