package solution;

import problem.ASVConfig;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.*;
import java.util.regex.Matcher;

public class MotionPlan {
    ASVConfig startConfig;
    ASVConfig endConfig;
    ProblemSpec ps;
    Tester tester = new Tester();

    double expansionAngle;
    double rotationAngle;


    /**
     * Constructor
     * @param start
     * @param endPoint
     * @param spec
     */
    public MotionPlan (ASVConfig start, Point2D endPoint, ProblemSpec spec) {
        this.startConfig = start;
        this.endConfig = validConfigOnPoint(start, endPoint);
        this.ps = spec;
    }

    public void setExpansionAngle(double expansionAngle) {
        this.expansionAngle = expansionAngle;
    }

    public void setRotationAngle(double rotationAngle) {
        this.rotationAngle = rotationAngle;
    }

    /**
     * Getting the path configs between start and end configs
     * @return
     */
    public List<ASVConfig> getPathConfigs() {
        List<ASVConfig> pathConfigs = new ArrayList<>();

        pathConfigs.addAll(getStraightPath(startConfig, endConfig));

        startConfig = pathConfigs.get(pathConfigs.size()-1);

        if (!sameConfig(startConfig, endConfig)) {
            pathConfigs.addAll(generateConfigs());
        }

        return pathConfigs;
    }

    /**
     * Generate all the configs if straight path does not exist
     * Generate until start is same config as goal
     * @return
     */
    private List<ASVConfig> generateConfigs() {
        List<ASVConfig> configsToReturn = new ArrayList<>();

        while (!sameConfig(startConfig, endConfig)) {
            configsToReturn.addAll(generateAlternateConfigs());
        }

        return configsToReturn;
    }


    /**
     * Generate alternate configs based on expansion rotation and forward movement
     * @return
     */
    private List<ASVConfig> generateAlternateConfigs() {
        List<ASVConfig> alternateConfigs = new ArrayList<>();

        ASVConfig nextStep = nextStep(startConfig, endConfig);

        int counter = 50;
        while (!validConfig(nextStep) && counter>0) {
            alternateConfigs.addAll(manipulateConfig(startConfig));

            if (!alternateConfigs.isEmpty()) {
                startConfig = alternateConfigs.get(alternateConfigs.size() - 1);
            }
            nextStep = nextStep(startConfig, endConfig);

            counter--;
        }


        alternateConfigs.add(nextStep);
        startConfig = nextStep;

        return alternateConfigs;
    }


    /**
     * Manipulation using rotation and expansion
     * @param startConfig
     * @return
     */
    private List<ASVConfig> manipulateConfig(ASVConfig startConfig) {
        List<ASVConfig> manipulated = new ArrayList<>();

        ASVConfig current = startConfig;

        ASVConfig rotated1 = rotateFirst(current, this.rotationAngle);
        ASVConfig rotated2 = rotateFirst(current, -this.rotationAngle);
        if (validConfig(rotated1)) {
            manipulated.add(rotated1);
            current = rotated1;
        }else if (validConfig(rotated2)) {
            manipulated.add(rotated2);
            current = rotated2;
        }

        List<ASVConfig> expanded = expand(current);

        if (!expanded.isEmpty()) {
            manipulated.addAll(expanded);
        }

        return manipulated;
    }

    private List<ASVConfig> expand(ASVConfig toExpand) {
        List<ASVConfig> expanded = new ArrayList<>();

        List<Point2D> points = toExpand.getASVPositions();

        for (int i = points.size()-2; i>0; i--) {
            points.set(i+1, rotatePoints(points.get(i+1), points.get(i), this.expansionAngle));

            ASVConfig onePoint = new ASVConfig(points);

            if (validConfig(onePoint)) {
                expanded.add(onePoint);
            }else {
                break;
            }

        }

        return expanded;
    }


    /**
     * Rotate ASVConfig
     * @param toRorate
     * @return
     */
    private ASVConfig rotateFirst(ASVConfig toRorate, double angle) {
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
        angle = Math.toRadians(angle);
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = ((toRotate.getX() - center.getX())*cos - (toRotate.getY() - center.getY())*sin) + center.getX();
        double y = ((toRotate.getX() - center.getX())*sin + (toRotate.getY() - center.getY())*cos) + center.getY();

        return new Point2D.Double(x,y);
    }


    private List<ASVConfig> getStraightPath(ASVConfig startConfig, ASVConfig endConfig) {
        List<ASVConfig> path = new ArrayList<>();

        ASVConfig current = startConfig;

        while (!sameConfig(current, endConfig)) {
            ASVConfig nextStep = nextStep(current, endConfig);

            if (validConfig(nextStep)) {
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
    private ASVConfig nextStep(ASVConfig currentConfig, ASVConfig nextConfig) {


        List<Point2D> nextStep = new ArrayList<>();
        for (int i = 0; i< currentConfig.getASVPositions().size(); i++) {
            nextStep.add(nextPoint(currentConfig.getASVPositions().get(i), nextConfig.getASVPositions().get(i), Tester.MAX_STEP));
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
     * Returns next config on a path point.
     * @param current
     * @param point
     * @return
     */
    private ASVConfig validConfigOnPoint(ASVConfig current, Point2D point) {
        List<Point2D> currentConfigs = current.getASVPositions();
        List<Point2D> transformed = new ArrayList<>(currentConfigs.size());
        transformed.add(point);

        for (int i = 0; i < currentConfigs.size()-1; i++) {
            Point2D p0 = currentConfigs.get(i);
            Point2D p1 = currentConfigs.get(i+1);

            double distance = Tester.MAX_BOOM_LENGTH;

            double angle = Math.atan2(p1.getY() - p0.getY(), p1.getX() - p0.getX());
            double x = point.getX() + (Math.cos(angle)*distance);
            double y = point.getY() + (Math.sin(angle)*distance);
            PathPoint thisPoint = new PathPoint(x,y);
            point = thisPoint;
            transformed.add(thisPoint);
        }

        return new ASVConfig(transformed);

    }


    /**
     * Checks if there is a collision straight line of two points
     * @param p1
     * @param p2
     * @return
     */
    private boolean hasCollision(Point2D p1, Point2D p2) {
        Line2D line = new Line2D.Double(p1, p2);

        for (int i = 0; i< ps.getObstacles().size(); i++) {
            if (line.intersects(ps.getObstacles().get(i).getRect())) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns if a config is same as another
     * @param cfg1
     * @param cfg2
     * @return
     */
    private boolean sameConfig(ASVConfig cfg1, ASVConfig cfg2) {
        return cfg1.maxDistance(cfg2) <= Tester.MAX_STEP;
    }

    /**
     * Tests if a given config is valid or not
     * @param config
     * @return
     */
    public boolean validConfig(ASVConfig config) {
        boolean test1 = !tester.hasCollision(config, ps.getObstacles());
        boolean test3 = tester.isConvex(config);
        boolean test4 = tester.hasEnoughArea(config);
        boolean test5 = tester.fitsBounds(config);

        return test1 && test3 && test4 && test5;
    }




}
