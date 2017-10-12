package solution;

import Solution2.MinHeuristicsComparator;
import problem.ASVConfig;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.*;

public class MotionSearch {

    private ASVConfig startConfig;
    private ASVConfig endConfig;
    private ProblemSpec ps;
    private Tester tester;

    private final double ROTATION_ANGLE = 1;
    private final double EXPANSION_ANGLE = 1;


    /**
     * Constructor
     * @param startConfig
     * @param nextPoint
     */
    public MotionSearch (ASVConfig startConfig, Point2D nextPoint, ProblemSpec ps, Tester tester) {
        this.startConfig = startConfig;
        this.ps = ps;
        this.tester = tester;
        this.endConfig = new ASVConfig(validConfigOnPoint(startConfig, nextPoint));
    }


    /**
     * Calls the breadth first search method and get path stack.
     * Convert the stack to list and return the list containing ASV configs
     * @return
     */
    public List<ASVConfig> getPathConfigs () {

        List<ASVConfig> pathConfig = new ArrayList<>();

        pathConfig.addAll(straightPath());

        if (!pathConfig.isEmpty()) {
            startConfig = pathConfig.get(pathConfig.size()-1);
        }

        if (sameConfig(startConfig, endConfig)) {
            return pathConfig;
        }

        List<Point2D> pointsOnLine = pointsOnLine(startConfig.getASVPositions().get(0), endConfig.getASVPositions().get(0), 0.01);

        int test = 10;

        for (int i = 0; i < pointsOnLine.size(); i++) {

            if (test < 1) {
                //return pathConfig;
            }
            test--;

            Point2D p = pointsOnLine.get(i);

            endConfig = validConfigOnPoint(startConfig, p);

            //pathConfig.addAll(giveMeConfigs(startConfig, endConfig));



            graphSearch(startConfig, endConfig);

            Stack<ASVConfig> pathStack = getPathStack();
            while (!pathStack.isEmpty()) {
                pathConfig.add(pathStack.pop());
            }


            startConfig = new ASVConfig(endConfig.getASVPositions());
        }

        return pathConfig;

    }

    /**
     * Returns the path stack by traversing back from end node to start node going through each parent
     * @return
     */
    private Stack<ASVConfig> getPathStack () {

        Stack<ASVConfig> pathStack = new Stack<>();

        ASVConfig currentConfig = endConfig;

        pathStack.push(currentConfig);

        while (currentConfig.hasParent()) {
            pathStack.push(currentConfig.getParent());
            currentConfig = currentConfig.getParent();
        }
        return pathStack;
    }


    /**
     * BFS algorithm for motion configs
     */
    private void graphSearch(ASVConfig startConfig, ASVConfig nextConfig) {

        startConfig.setDistanceFromStart(0);
        startConfig.setDistanceToGoal(startConfig.maxDistance(nextConfig));

        //Comparator and priority queue
        Comparator<ASVConfig> comparator = new MinHeuristicsComparator();
        PriorityQueue<ASVConfig> queue = new PriorityQueue<>(comparator);

        ASVConfig visitng = startConfig;

        queue.add(startConfig);

        int test = 10000;
        while (!queue.isEmpty() && test>0) {



            ASVConfig currentNode = queue.poll();

            visitng = currentNode;

            List<ASVConfig> childConfigs = getChildConfigs(currentNode, nextConfig);


            for (ASVConfig c : childConfigs) {

                c.setParent(currentNode);

                if (sameConfig(c, nextConfig)) {
                    return;
                }
                c.setDistanceFromStart(currentNode.getDistanceFromStart()+currentNode.maxDistance(c));
                c.setDistanceToGoal(c.maxDistance(nextConfig));

                if (validHeuristics(c, nextConfig)) {
                    queue.add(c);
                }
            }

            test--;
        }

        endConfig.setParent(visitng);
    }


    /**
     * Checks if two nodes are same or not.
     * @param n1
     * @param n2
     * @return
     */
    private boolean sameNode (StepNode n1, StepNode n2) {
        return sameConfig(n1.getConfig(), n2.getConfig());
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
     * Returns if a config is same as another
     * @param cfg1
     * @param cfg2
     * @return
     */
    private boolean sameVisitedConfig(ASVConfig cfg1, ASVConfig cfg2) {
        return cfg1.maxDistance(cfg2) <= Tester.DEFAULT_MAX_ERROR;
    }

    /**
     * Generate child configs
     * Returns the list of every possible movement
     * @param config
     * @return
     */
    private List<ASVConfig> getChildConfigs(ASVConfig config, ASVConfig nextConfig) {

        List<ASVConfig> childConfigs = new ArrayList<>();

        ASVConfig lineStepMovement = nextStep(config, nextConfig);
        lineStepMovement.setDistanceFromStart(config.getDistanceFromStart()+config.maxDistance(lineStepMovement));
        lineStepMovement.setDistanceToGoal(lineStepMovement.maxDistance(nextConfig));
        if (validConfig(lineStepMovement)) {
            childConfigs.add(lineStepMovement);
        }


        ASVConfig rotationFirstClock = rotateFirst(config, -ROTATION_ANGLE);
        rotationFirstClock.setDistanceFromStart(config.getDistanceFromStart()+config.maxDistance(rotationFirstClock));
        rotationFirstClock.setDistanceToGoal(rotationFirstClock.maxDistance(nextConfig));
        if (validConfig(rotationFirstClock) && validHeuristics(rotationFirstClock, nextConfig)) {
            childConfigs.add(rotationFirstClock);
        }

        ASVConfig rotationLastClock = rotateLast(config, -ROTATION_ANGLE);
        rotationLastClock.setDistanceFromStart(config.getDistanceFromStart()+config.maxDistance(rotationLastClock));
        rotationLastClock.setDistanceToGoal(rotationLastClock.maxDistance(nextConfig));
        if (validConfig(rotationLastClock) && validHeuristics(rotationLastClock, nextConfig)) {
            //childConfigs.add(rotationFirstClock);
        }

        ASVConfig rotationFirstAnti = rotateFirst(config, ROTATION_ANGLE);
        rotationFirstAnti.setDistanceFromStart(config.getDistanceFromStart()+config.maxDistance(rotationFirstAnti));
        rotationFirstAnti.setDistanceToGoal(rotationFirstAnti.maxDistance(nextConfig));
        if (validConfig(rotationFirstAnti) && validHeuristics(rotationFirstAnti, nextConfig)) {
            //childConfigs.add(rotationFirstAnti);
        }

        ASVConfig rotationLastAnti = rotateLast(config, ROTATION_ANGLE);
        rotationLastAnti.setDistanceFromStart(config.getDistanceFromStart()+config.maxDistance(rotationLastAnti));
        rotationLastAnti.setDistanceToGoal(rotationLastAnti.maxDistance(nextConfig));
        if (validConfig(rotationLastAnti) && validHeuristics(rotationLastAnti, nextConfig)) {
            //childConfigs.add(rotationLastAnti);
        }

        List<ASVConfig> expansionType1 = expand(config, -EXPANSION_ANGLE);
        if (validConfig(expansionType1.get(0)) && validConfig(expansionType1.get(expansionType1.size()-1))) {
            // childConfigs.addAll(expansionType1);
        }


        List<ASVConfig> expansionType2 = expand(config, EXPANSION_ANGLE);
        if (validConfig(expansionType2.get(0)) && validConfig(expansionType2.get(expansionType2.size()-1))) {
            //childConfigs.addAll(expansionType2);
        }

        ASVConfig moveUp = move(config, 0.001,0,0,0);
        moveUp.setDistanceFromStart(config.getDistanceFromStart()+config.maxDistance(moveUp));
        moveUp.setDistanceToGoal(moveUp.maxDistance(nextConfig));
        if (validConfig(moveUp)  && validHeuristics(moveUp, nextConfig)) {
            //childConfigs.add(moveUp);
        }

        ASVConfig moveLeft = move(config, 0,0,0.001,0);
        moveLeft.setDistanceFromStart(config.getDistanceFromStart()+config.maxDistance(moveLeft));
        moveLeft.setDistanceToGoal(moveLeft.maxDistance(nextConfig));
        if (validConfig(moveLeft)  && validHeuristics(moveLeft, nextConfig)) {
            childConfigs.add(moveLeft);
        }

        ASVConfig moveDown = move(config, 0,0.001,0,0);
        moveDown.setDistanceFromStart(config.getDistanceFromStart()+config.maxDistance(moveDown));
        moveDown.setDistanceToGoal(moveDown.maxDistance(nextConfig));
        if (validConfig(moveDown) && validHeuristics(moveDown, nextConfig)) {
            //childConfigs.add(moveDown);
        }

        ASVConfig moveRight = move(config, 0, 0,0,0.001);
        moveRight.setDistanceFromStart(config.getDistanceFromStart()+config.maxDistance(moveRight));
        moveRight.setDistanceToGoal(moveRight.maxDistance(nextConfig));
        if (validConfig(moveRight)  && validHeuristics(moveRight, nextConfig)) {
            //childConfigs.add(moveRight);
        }

        List<ASVConfig> contractionType1 = contract(config, -EXPANSION_ANGLE);
        if (validConfig(contractionType1.get(0)) && validConfig(contractionType1.get(contractionType1.size()-1))) {
            //childConfigs.addAll(contractionType1);
        }

        List<ASVConfig> contractionType2 = contract(config, EXPANSION_ANGLE);
        if (validConfig(contractionType2.get(0)) && validConfig(contractionType2.get(contractionType2.size()-1))) {
            //childConfigs.addAll(contractionType2);
        }

        return childConfigs;

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

        for (int i = 1; i < positions.size()-1; i++) {
            for (int j = i + 1; j < positions.size(); j++) {
                positions.set(j, rotatePoints(positions.get(j), positions.get(i), angle));
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
    private List<ASVConfig> contract(ASVConfig toContract, double angle) {
        List<ASVConfig> contracted = new ArrayList<>();

        List<Point2D> positions = toContract.getASVPositions();

        for (int i=1; i < positions.size(); i++) {
            for (int j=0; j<i; j++) {
                positions.set(j, rotatePoints(positions.get(j), positions.get(i), angle));
            }
            contracted.add(new ASVConfig(positions));
        }
        return contracted;
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
     * Returns next step of ASV configs on a line with a step 0.001
     * @param currentConfig
     * @return
     */
    private ASVConfig nextStep(ASVConfig currentConfig) {

        ASVConfig nextConfig = nextConfig(currentConfig, endConfig.getASVPositions().get(0));

        List<Point2D> nextStep = new ArrayList<>();
        for (int i = 0; i< currentConfig.getASVPositions().size(); i++) {
            nextStep.add(nextPoint(currentConfig.getASVPositions().get(i), nextConfig.getASVPositions().get(i), Tester.MAX_STEP));
        }


        return new ASVConfig(nextStep);
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

    private boolean straightLine () {
        for (int i=0; i<startConfig.getASVCount(); i++) {
            Point2D p1 = startConfig.getASVPositions().get(i);
            Point2D p2 = endConfig.getASVPositions().get(i);
            if (hasCollision(p1, p2)) {
                return false;
            }
        }
        return true;
    }

    private boolean hasCollision(Point2D p1, Point2D p2) {
        Line2D line = new Line2D.Double(p1, p2);

        for (int i = 0; i< ps.getObstacles().size(); i++) {
            if (line.intersects(ps.getObstacles().get(i).getRect())) {
                return true;
            }
        }
        return false;
    }

    private List<ASVConfig> straightPath() {
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

    private ASVConfig validConfigOnPoint(ASVConfig current, Point2D point) {
        List<ASVConfig> validConfig = new ArrayList<>();
        validConfig.add(nextConfig(current, point));

        ASVConfig currentConfig = validConfig.get(0);

        if (validConfig(currentConfig)) {
            return currentConfig;
        }

        /*
        while (validExpansion(currentConfig)) {
            validConfig.add(currentConfig);

            List<ASVConfig> expanded = expand(currentConfig, -EXPANSION_ANGLE);
            currentConfig = expanded.get(expanded.size()-1);
        }*/

        //currentConfig = validConfig.get(validConfig.size()-1);


        while (!validConfig(currentConfig)) {
            ASVConfig rotationFirst = rotateFirst(currentConfig, -ROTATION_ANGLE);
            currentConfig = rotationFirst;

        }

        return currentConfig;
    }

    /*
    private ASVConfig validConfigOnPoint(ASVConfig current, Point2D point) {
        List<ASVConfig> validConfig = new ArrayList<>();
        validConfig.add(nextConfig(current, point));

        ASVConfig currentConfig = validConfig.get(0);

        while (!validConfig(currentConfig)) {
            ASVConfig rotated = rotateFirst(currentConfig, -ROTATION_ANGLE);
            while (!sameVisitedConfig(currentConfig, rotated)) {
                rotated = rotateFirst(rotated, -ROTATION_ANGLE);
                if (validConfig(rotated)) {
                    return rotated;
                }
            }
            currentConfig = rotated;
            List<ASVConfig> expanded = expand(currentConfig, -EXPANSION_ANGLE);
            if (validExpansion(expanded.get(expanded.size()-1))) {
                currentConfig = expanded.get(expanded.size() - 1);
            }

        }
        return currentConfig;
    }
     */





    private boolean validHeuristics(ASVConfig c, ASVConfig endConfig) {
        return c.getDistanceFromStart() <= 1.5 * c.maxDistance(endConfig);
    }

    /**
     * Generate all points in a line
     * @param p1
     * @param p2
     */
    public ArrayList<Point2D> pointsOnLine(Point2D p1, Point2D p2, double stepSize) {

        double diffX = p2.getX() - p1.getX();
        double diffY = p2.getY() - p1.getY();
        double numberOfPoints = p1.distance(p2);

        double intervalX = diffX / (numberOfPoints);
        double intervalY = diffY / (numberOfPoints);

        ArrayList<Point2D> points = new ArrayList<>();
        if (numberOfPoints == 0) {
            return points;
        }

        for (double i = 0; i < numberOfPoints+stepSize; i=i+stepSize) {
            double x = p1.getX()+intervalX*i;
            double y = p1.getY()+intervalY*i;
            points.add(new Point2D.Double(x,y));
        }

        return points;

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

    /**
     * Tests if a given config is valid or not
     * @param config
     * @return
     */
    public boolean validExpansion(ASVConfig config) {
        boolean test3 = tester.isConvex(config);
        boolean test4 = tester.hasEnoughArea(config);
        boolean test5 = tester.fitsBounds(config);

        return test3 && test4 && test5;
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
            angle = -0.4;
        }else {
            angle = 0.4;
        }

        int test = 100;
        for (int i=0; i< positions.size()-1; i++) {
            while (!samePoint(positions.get(i+1), goalConfig.getASVPositions().get(i+1)) && test>0) {
                for (int j = i + 1; j < positions.size(); j++) {
                    positions.set(j, rotatePoints(positions.get(j), positions.get(i), angle));
                }
                transformed.add(new ASVConfig(positions));
                test--;

            }
        }



        return transformed;
    }

    private boolean samePoint(Point2D point2D, Point2D point2D1) {
        return point2D.distance(point2D1) <= Tester.DEFAULT_MAX_ERROR;
    }


    private List<ASVConfig> giveMeConfigs (ASVConfig startConfig, ASVConfig endConfig) {
        List<ASVConfig> configsToReturn = new ArrayList<>();

        ASVConfig nextStep = nextStep(startConfig, endConfig);

        ASVConfig currentConfig = startConfig;

        int test = 1000;

        while (!validConfig(nextStep) && test>0) {
            ASVConfig left = move(startConfig, 0, 0, 0, 0.001);

            if (validConfig(left)) {
                currentConfig = left;
                configsToReturn.add(left);
            }

            ASVConfig rotated = rotateFirst(currentConfig, -ROTATION_ANGLE);

            if (validConfig(rotated)) {
                currentConfig = rotated;
                configsToReturn.add(rotated);
            }

            ASVConfig leftAgain = move(startConfig, 0, 0, 0, 0.001);

            if (validConfig(leftAgain)) {
                currentConfig = leftAgain;
                configsToReturn.add(leftAgain);
            }

            List<ASVConfig> expanded = expand(currentConfig, -EXPANSION_ANGLE);

            if (validConfig(expanded.get(expanded.size()-1)) && validConfig(expanded.get(0))) {
                configsToReturn.addAll(expanded);
                currentConfig = expanded.get(expanded.size()-1);
            }

            nextStep = nextStep(currentConfig, endConfig);

            test--;

        }


        return configsToReturn;
    }


}
