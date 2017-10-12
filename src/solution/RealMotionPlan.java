package solution;

import Solution2.MinHeuristicsComparator;
import problem.ASVConfig;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Point2D;
import java.util.*;

public class RealMotionPlan {

    ASVConfig startConfig;
    Point2D nextPoint;
    Tester tester = new Tester();
    ProblemSpec ps;
    int direction;

    /**
     * Constructor
     * @param startConfig
     * @param nextPoint
     */
    public RealMotionPlan(ASVConfig startConfig, Point2D nextPoint, ProblemSpec ps, int direction) {
        this.startConfig = startConfig;
        this.nextPoint = nextPoint;
        this.ps = ps;
        this.direction = direction;
    }

    /**
     * Returns all the path configurations between start config and next point given
     * @return
     */
    public List<ASVConfig> getPathConfigs() {
        List<ASVConfig> pathConfigs = new ArrayList<>();
        pathConfigs.add(startConfig);

        List<Point2D> pointsOnPath = allPointsOnPath(startConfig.getASVPositions().get(0), nextPoint);

        for (int i = 0; i < pointsOnPath.size(); i++) {
            pathConfigs.addAll(oneStepConfigs(pathConfigs.get(pathConfigs.size()-1), pointsOnPath.get(i)));
        }

        return pathConfigs;
    }


    /**
     * Generate possible valid configs on a point by manipulating original angles
     * @param startConfig
     * @param point
     * @return
     *
    private List<ASVConfig> oneStepConfigs(ASVConfig startConfig, Point2D point) {
        List<ASVConfig> configs = new ArrayList<>();
        configs.add(startConfig);

        List<Point2D> points = new ArrayList<>();
        points.add(point);

        double[] currentAngles = startConfig.getAngles();

        int test = 500;
        while (!samePoint(configs.get(configs.size()-1).getASVPositions().get(0), point) && test>0) {

            for (int i = 1; i < startConfig.getASVCount(); i++) {
                double[] anglesInLoop = currentAngles;
                double angle = generateRandomAngle(anglesInLoop[i]);
                anglesInLoop[i] = angle;

                ASVConfig configInLoop = new ASVConfig(generateConfig(points.get(0), anglesInLoop));

                if (validConfig(configInLoop)) {
                    configs.add(configInLoop);
                    currentAngles = configInLoop.getAngles();
                } else {
                    break;
                }

            }

            test--;
        }

        //System.out.println(configs.size() + "  ----- Itne zada configs is dafa");

        return configs;
    }
    */


    private List<ASVConfig> oneStepConfigs(ASVConfig startConfig, Point2D point) {
        List<ASVConfig> configs = new ArrayList<>();

        ASVConfig nextConfigToCheck = generateConfig(point, startConfig.getAngles());

        if (!validConfig(nextConfigToCheck, startConfig)) {
            configs.addAll(graphSearch(startConfig, point));
        }else {
            configs.add(nextConfigToCheck);
        }

        return configs;
    }

    /**
     * A* algorithm for motion configs
     */
    private List<ASVConfig> graphSearch(ASVConfig startConfig, Point2D nextPoint) {

        startConfig.setDistanceFromStart(0);
        startConfig.setDistanceToGoal(startConfig.getASVPositions().get(0).distance(nextPoint));

        //Comparator and priority queue
        Comparator<ASVConfig> comparator = new MinHeuristicsComparator();
        PriorityQueue<ASVConfig> queue = new PriorityQueue<>(comparator);

        ASVConfig visiting = startConfig;

        queue.add(startConfig);

        int test = 500;
        while (!queue.isEmpty() && test>0) {

            ASVConfig currentNode = queue.poll();
            visiting = currentNode;

            List<ASVConfig> childConfigs = getChildConfigs(currentNode, nextPoint);

            for (ASVConfig c : childConfigs) {

                c.setParent(currentNode);

                if (samePoint(c.getASVPositions().get(0), nextPoint)) {
                    visiting = c;
                    break;
                }

                c.setDistanceFromStart(currentNode.getDistanceFromStart()+currentNode.getASVPositions().get(0).distance(c.getASVPositions().get(0)));
                c.setDistanceToGoal(c.getASVPositions().get(0).distance(nextPoint));

                if (validHeuristics(c.getASVPositions().get(0), nextPoint)) {
                    queue.add(c);
                }
            }

            test--;
        }

        Stack<ASVConfig> pathStack = new Stack<>();
        pathStack.push(visiting);
        while (visiting.hasParent()) {
            pathStack.push(visiting.getParent());
            visiting = visiting.getParent();
        }

        List<ASVConfig> configsToReturn = new ArrayList<>();

        while (!pathStack.isEmpty()) {
            configsToReturn.add(pathStack.pop());
        }

        return configsToReturn;
    }


    /**
     * Generate all possible child configs
     * @param config
     * @param point
     * @return
     */
    private List<ASVConfig> getChildConfigs(ASVConfig config, Point2D point) {

        List<ASVConfig> configsToReturn = new ArrayList<>();

        double[] angles = config.getAngles();

        for (int i=1; i<angles.length; i++) {
            double[] localAngles = angles;
            localAngles[i] += (Tester.DEFAULT_EXP_ANGLE*-direction);
            ASVConfig configCheck = generateConfig(config.getASVPositions().get(0), localAngles);
            ASVConfig nextConfig = generateConfig(point, localAngles);
            if (validConfig(configCheck, config)) {
                configsToReturn.add(configCheck);
            }
            if (validConfig(nextConfig, config)) {
                configsToReturn.add(nextConfig);
            }
        }

        if (!configsToReturn.isEmpty()) {
            config = configsToReturn.get(configsToReturn.size() - 1);

            ASVConfig next = generateConfig(point, config.getAngles());
            if (validConfig(next, config)) {
                configsToReturn.add(next);
            }

        }

        ASVConfig left = move(config, 0,0,0,0.001);
        if (validConfig(left, config)) {
            configsToReturn.add(left);
        }

        ASVConfig up = move(config, 0.001,0,0,0);
        if (validConfig(up, config)) {
            configsToReturn.add(up);
        }

        ASVConfig down = move(config, 0,0.001,0,0);
        if (validConfig(down, config)) {
            configsToReturn.add(down);
        }

        ASVConfig right = move(config, 0,0,0.001,0);
        if (validConfig(right, config)) {
            configsToReturn.add(right);
        }


        return configsToReturn;
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
     * Checks if two points are same or not
     * @param point2D
     * @param point2D1
     * @return
     */
    private boolean samePoint(Point2D point2D, Point2D point2D1) {
        return point2D.distance(point2D1) <= Tester.DEFAULT_MAX_ERROR;
    }

    /**
     * Generate a configuration based on first point and all the angles
     * @param point
     * @param angles
     * @return
     */
    private ASVConfig generateConfig(Point2D point, double[] angles) {
        List<Point2D> configPoints = new ArrayList<>();
        configPoints.add(point);

        for (int i = 1; i < angles.length; i++) {
            double x = configPoints.get(configPoints.size()-1).getX() + (Math.cos(angles[i])*Tester.MAX_BOOM_LENGTH);
            double y = configPoints.get(configPoints.size()-1).getY() + (Math.sin(angles[i])*Tester.MAX_BOOM_LENGTH);
            PathPoint thisPoint = new PathPoint(x,y);
            configPoints.add(thisPoint);
        }
        return new ASVConfig(configPoints);
    }

    /**
     * Generate random angle closer to given angle
     * @param angle
     * @return
     */
    private double generateRandomAngle(double angle, int direction) {

        Random r = new Random();
        double min = angle;
        double max = angle;
        if (direction < 0) {
            min = angle - Tester.DEFAULT_EXP_ANGLE;
        }else {
            max = angle + Tester.DEFAULT_EXP_ANGLE;
        }

        double random = min + r.nextDouble() * (max - min);

        return random;
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

        for (double i = 0; i < numberOfPoints+Tester.MAX_STEP; i=i+Tester.MAX_STEP) {
            double x = p1.getX()+intervalX*i;
            double y = p1.getY()+intervalY*i;
            points.add(new Point2D.Double(x,y));
        }
        return points;
    }

    private boolean validHeuristics(Point2D p1, Point2D p2) {
        return p1.distance(p2) <= 1.5 * Tester.MAX_STEP;
    }

    /**
     * Tests if a given config is valid or not
     * @param config
     * @return
     */
    public boolean validConfig(ASVConfig config, ASVConfig previous) {
        boolean test1 = !tester.hasCollision(config, ps.getObstacles());
        boolean test2 = tester.isValidStep(previous, config);
        boolean test3 = tester.isConvex(config);
        boolean test4 = tester.hasEnoughArea(config);
        boolean test5 = tester.fitsBounds(config);

        return test1 && test2 && test3 && test4 && test5;
    }

}
