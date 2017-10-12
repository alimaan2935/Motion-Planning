package solution;

import org.omg.PortableInterceptor.SYSTEM_EXCEPTION;
import problem.ASVConfig;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Stack;

public class Motion {
    private ProblemSpec ps;
    private Stack<PathPoint> path;
    private Tester test = new Tester();

    private final double ROTATION_ANGLE = 0.2;
    private final double EXPANSION_ANGLE = 0.2;


    /**
     * Constructor
     * @param ps problem spec
     * @param path stack path points
     */
    public Motion (ProblemSpec ps, Stack<PathPoint> path) {
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


        while (path.size() > 3) {
            Point2D nextPoint = path.pop();
            ASVConfig current = completePath.get(completePath.size()-1);
            completePath.addAll(transformation(current, nextPoint));

            current = completePath.get(completePath.size()-1);

            if (samePoint(current.getASVPositions().get(0), nextPoint) && path.size()>0) {
                //completePath.addAll(transformTillGoal(current, validConfigOnPoint(startConfig, nextPoint)));
            }

        }

        ASVConfig goalConfigToTransform = nextConfig(completePath.get(completePath.size()-1), goalConfig.getASVPositions().get(0));
        completePath.add(goalConfigToTransform);
        completePath.addAll(transformTillGoal(goalConfigToTransform, goalConfig));
        completePath.add(goalConfig);
        return completePath;
    }

    /**
     * Transform the asv configs until it reaches next point
     * @param current ASVConfig
     * @param nextPoint Point2D
     * @return list of transformed ASVConfigs
     */
    private List<ASVConfig> transformation (ASVConfig current, Point2D nextPoint) {
        List<ASVConfig> transformedList = new ArrayList<>();

        ASVConfig currentConfig = current;

        int test = 1000;
        while (!samePoint(currentConfig.getASVPositions().get(0), nextPoint) && test>0) {
            transformedList.addAll(doTransformation(currentConfig, nextPoint));
            if (!transformedList.isEmpty()) {
                currentConfig = transformedList.get(transformedList.size() - 1);
            }
            test--;
            //System.out.println("Current: " + currentConfig.getASVPositions().get(0) + "  Next Point: " + nextPoint + " Distance: " + nextPoint.distance(currentConfig.getASVPositions().get(0)));

        }
        return transformedList;
    }

    /**
     * Make the decision of transformation and do the transformation
     * If nextStep is in collision, then set next step as rotation. If rotation step has collision,
     * do expand step with collision, convexity, and minimum area checks.
     *
     * @param currentConfig
     * @param nextPoint
     * @return list of transformed ASVConfigs
     */
    private List<ASVConfig> doTransformation (ASVConfig currentConfig, Point2D nextPoint) {
        List<ASVConfig> transformedList = new ArrayList<>();

        ASVConfig current = currentConfig;

        ASVConfig nextPointConfig = nextConfig(current, nextPoint);
        ASVConfig nextStep = nextStep(current, nextPointConfig);

        if (!linMovTest(nextStep)) {

            // Do Left Movement
            ASVConfig moveLeft = moveLeft(current);
            if (linMovTest(moveLeft)) {
                transformedList.add(moveLeft);
                current = moveLeft;
            }

            // Do rotation around first
            ASVConfig rotatedFirst = rotateFirst(current);
            if (rotationTest(rotatedFirst)) {
                transformedList.add(rotatedFirst);
                current = rotatedFirst;
            }

            //  Do Expansion
            List<ASVConfig> expanded = expand(current);
            if (!expanded.isEmpty()) {
                if (expansionTest(expanded.get(0)) && expansionTest(expanded.get(expanded.size() - 1))) {
                    transformedList.addAll(expanded);
                    current = transformedList.get(transformedList.size() - 1);
                }
            }

            // Do rotation around last
            ASVConfig rotatedLast = rotateLast(current);
            if (rotationTest(rotatedLast)) {
                transformedList.add(rotatedLast);
                current = rotatedLast;
            }

            // Do up or down based on distance from next point
            ASVConfig moveVertical = moveVertical(current, nextPoint);
            if (linMovTest(moveVertical)) {
                transformedList.add(moveVertical);
                current = moveVertical;
            }


        }else {
            transformedList.add(nextStep);
        }
        return transformedList;
    }

    private boolean nearPoint (Point2D p1, Point2D p2) {
        return p1.distance(p2) <= 0.1;
    }

    /**
     * Returns if a point is same as another.
     * @param p1
     * @param p2
     * @return
     */
    private boolean samePoint(Point2D p1, Point2D p2) {
        return p1.distance(p2) <= Tester.MAX_STEP;
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


    private ASVConfig validConfigOnPoint (ASVConfig start, Point2D point) {
        ASVConfig validConfig = nextConfig(start, point);

        while (!expansionTest(validConfig)) {
            validConfig = rotateFirst(validConfig);
        }

        return validConfig;
    }

    /**
     * Returns next config on a path point.
     * @param current
     * @param point
     * @return
     */
    private ASVConfig nextConfig(ASVConfig current, Point2D point) {
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
     * Returns next step of ASV configs on a line with a step 0.001
     * @param currentConfig
     * @param nextConfig
     * @return
     */
    private ASVConfig nextStep(ASVConfig currentConfig, ASVConfig nextConfig) {

        List<Point2D> nextStep = new ArrayList<>();
        for (int i = 0; i< currentConfig.getASVPositions().size(); i++) {
            nextStep.add(nextPoint(currentConfig.getASVPositions().get(i), nextConfig.getASVPositions().get(i)));
        }


        return new ASVConfig(nextStep);
    }

    /**
     * Returns next point on the line by the interval 0.001
     * @param p1
     * @param p2
     * @return
     */
    private Point2D nextPoint(Point2D p1, Point2D p2) {
        double diffX = p2.getX() - p1.getX();
        double diffY = p2.getY() - p1.getY();
        double numberOfPoints = p1.distance(p2);

        double intervalX = diffX / (numberOfPoints);
        double intervalY = diffY / (numberOfPoints);

        Point2D nextPoint = p2;
        if (numberOfPoints <= Tester.MAX_STEP) {
            return nextPoint;
        }

        double x = p1.getX()+intervalX* Tester.MAX_STEP;
        double y = p1.getY()+intervalY* Tester.MAX_STEP;

        return new PathPoint(x,y);
    }

    /**
     * Rotate ASVConfig one degree
     * @param toRorate
     * @return
     */
    private ASVConfig rotateFirst(ASVConfig toRorate) {
        ASVConfig clockWise = rotateFirst(toRorate, -this.ROTATION_ANGLE);
        ASVConfig counterClockWise = rotateFirst(toRorate, this.ROTATION_ANGLE);

        if (rotationTest(clockWise)) {
            return clockWise;
        }else {
            return counterClockWise;
        }
    }

    /**
     * Rotate ASVConfig one degree
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
     * Rotate ASVConfig one degree
     * @param toRorate
     * @return
     */
    private ASVConfig rotateLast(ASVConfig toRorate) {
        ASVConfig clockWise = rotateLast(toRorate, -this.ROTATION_ANGLE);
        ASVConfig counterClockWise = rotateLast(toRorate, this.ROTATION_ANGLE);

        if (rotationTest(clockWise)) {
            return clockWise;
        }else {
            return counterClockWise;
        }
    }

    /**
     * Rotate ASVConfig one degree
     * @param toRorate
     * @return
     */
    private ASVConfig rotateLast(ASVConfig toRorate, double angle) {
        List<Point2D> rotated = toRorate.getASVPositions();
        for (int i=0; i<rotated.size()-1; i++) {
            rotated.set(i, rotatePoints(rotated.get(i), rotated.get(rotated.size()-1), angle));
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
        angle = angle * Math.PI/180;
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = ((toRotate.getX() - center.getX())*cos - (toRotate.getY() - center.getY())*sin) + center.getX();
        double y = ((toRotate.getX() - center.getX())*sin + (toRotate.getY() - center.getY())*cos) + center.getY();

        return new Point2D.Double(x,y);
    }



    /**
     * Do linear transformation of ASV config
     * @param config
     * @return
     */
    private ASVConfig moveLeft (ASVConfig config) {
        ASVConfig left = move(config, 0,0,0, Tester.MAX_STEP);
        return left;
    }

    private ASVConfig moveVertical(ASVConfig config, Point2D nextPoint) {
        ASVConfig up = move(config, Tester.MAX_STEP, 0,0,0);
        double dUp = up.getASVPositions().get(0).distance(nextPoint);

        ASVConfig down = move(config, 0, Tester.MAX_STEP,0,0);
        double dDown = down.getASVPositions().get(0).distance(nextPoint);

        if (dUp < dDown && linMovTest(up)) {
            return up;
        }else {
            return down;
        }
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
    private List<ASVConfig> expand(ASVConfig toExpand) {
        List<ASVConfig> expanded = new ArrayList<>();

        List<Point2D> positions = toExpand.getASVPositions();

        for (int i = 1; i < positions.size()-1; i++) {
            for (int j = i + 1; j < positions.size(); j++) {
                positions.set(j, rotatePoints(positions.get(j), positions.get(i), -this.EXPANSION_ANGLE));
            }
            expanded.add(new ASVConfig(positions));
        }
        return expanded;
    }


    /**
     * Do the contraction
     * @param toContract
     * @return
     */
    private List<ASVConfig> contract(ASVConfig toContract) {
        List<ASVConfig> contracted = new ArrayList<>();

        List<Point2D> positions = toContract.getASVPositions();

        for (int i=1; i < positions.size(); i++) {
            for (int j=0; j<i; j++) {
                positions.set(j, rotatePoints(positions.get(j), positions.get(i), -this.EXPANSION_ANGLE));
            }
            contracted.add(new ASVConfig(positions));
        }
        return contracted;
    }


    /**
     * Transform a configuration till it reaches the goal config.
     * @param toTransform
     * @param goalConfig
     * @return
     */
    private List<ASVConfig> transformTillGoal (ASVConfig toTransform, ASVConfig goalConfig) {
        List<ASVConfig> transformed = new ArrayList<>();
        transformed.add(toTransform);

        List<Point2D> positions = toTransform.getASVPositions();

        Point2D rotateTest = rotatePoints(positions.get(1), positions.get(0), -1);
        double originalDist = goalConfig.getASVPositions().get(1).distance(positions.get(1));
        double rotatedDist = goalConfig.getASVPositions().get(1).distance(rotateTest);

        double angle;


        if (originalDist > rotatedDist) {
            angle = -this.ROTATION_ANGLE;
        }else {
            angle = this.ROTATION_ANGLE;
        }

        while (!sameConfig(transformed.get(transformed.size()-1), goalConfig)) {

            ASVConfig rotated = rotateTillGoal(transformed.get(transformed.size()-1), goalConfig, angle);
            if (rotationTest(rotated)) {
                transformed.add(rotated);
            }else {
                transformed.addAll(contract(transformed.get(transformed.size()-1)));

                ASVConfig nextPointConfig = nextConfig(transformed.get(transformed.size()-1), goalConfig.getASVPositions().get(0));
                ASVConfig current = transformed.get(transformed.size()-1);
                while (!samePoint(current.getASVPositions().get(0), nextPointConfig.getASVPositions().get(0))) {
                    transformed.add(nextStep(current, nextPointConfig));
                    current = transformed.get(transformed.size()-1);
                }
            }
        }

        return transformed;
    }


    private ASVConfig rotateTillGoal(ASVConfig toRotate, ASVConfig goal, double angle) {

        int indexToStart = 0;

        for (int i = 1; i< toRotate.getASVCount(); i++) {
            if (samePoint(toRotate.getASVPositions().get(i), goal.getASVPositions().get(i))) {
                indexToStart = i;
            }
        }

        List<Point2D> positions = toRotate.getASVPositions();

        for (int j = indexToStart + 1; j < positions.size(); j++) {
            positions.set(j, rotatePoints(positions.get(j), positions.get(indexToStart), angle));
        }

        return new ASVConfig(positions);

    }


    /**
     * Test to be performed on each linear movement towards next point.
     * @param next
     * @return
     */
    private boolean linMovTest (ASVConfig next) {
        boolean test1 = !test.hasCollision(next, ps.getObstacles());
        boolean test2 = test.fitsBounds(next);

        return test1 && test2;
    }

    /**
     * Test to be performed on each rotation
     * @param next
     * @return
     */
    private boolean rotationTest (ASVConfig next) {
        boolean test1 = !test.hasCollision(next, ps.getObstacles());
        boolean test3 = test.fitsBounds(next);

        return test1 && test3;
    }

    /**
     * Test to be performed on each expansion step
     * @param next
     * @return
     */
    private boolean expansionTest(ASVConfig next) {
        boolean test1 = !test.hasCollision(next, ps.getObstacles());
        boolean test3 = test.isConvex(next);
        boolean test4 = test.hasEnoughArea(next);
        boolean test5 = test.fitsBounds(next);

        return test1 && test3 && test4 && test5;
    }

    /**
     * Tests if a given config is valid or not
     * @param config
     * @return
     */
    public boolean validConfig(ASVConfig config) {
        boolean test1 = !test.hasCollision(config, ps.getObstacles());
        boolean test3 = test.isConvex(config);
        boolean test4 = test.hasEnoughArea(config);
        boolean test5 = test.fitsBounds(config);

        return test1 && test3 && test4 && test5;
    }

}
