package solution;

import problem.ASVConfig;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Stack;

public class NewSolution {
    private ProblemSpec ps;
    private Stack<PathPoint> path;
    private Tester test = new Tester();




    /**
     * Constructor
     * @param ps problem spec
     * @param path stack path points
     */
    public NewSolution (ProblemSpec ps, Stack<PathPoint> path) {
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

        int test = 100;

        while (path.size() > 0 && test>0) {
            Point2D nextPoint = path.pop();
            ASVConfig current = completePath.get(completePath.size()-1);
            completePath.addAll(transformation(current, nextPoint));
            test--;
        }

        ASVConfig goalCOnfigToTransform = nextConfig(completePath.get(completePath.size()-1), goalConfig.getASVPositions().get(0));
        completePath.add(goalCOnfigToTransform);
        completePath.addAll(transformTillGoal(goalCOnfigToTransform, goalConfig));
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

        int test = 1500;
        while (!samePoint(currentConfig.getASVPositions().get(0), nextPoint) && test>0) {
            transformedList.addAll(doTransformation(currentConfig, nextPoint));
            currentConfig = transformedList.get(transformedList.size()-1);
            test--;
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
            ASVConfig nextStepToTake = rotate(current, -0.4);
            if (!rotationTest(nextStepToTake)) {
                transformedList.addAll(expand(nextStep));
            }else {
             transformedList.add(nextStepToTake);
            }
        }else {
            transformedList.add(nextStep);
        }


        return transformedList;
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

    /**
     * Do linear transformation of ASV config
     * @param config
     * @return
     */
    private ASVConfig linearTransformation (ASVConfig config) {

        ASVConfig moved = move(config, 0,0,0,0.001);
        if (linMovTest(moved)) {
            return moved;
        }

        //moved = move(config, 0,0.001,0,0);
        if (linMovTest(moved)) {
            return moved;
        }

        //moved = move(config, 0,0,0.001,0);
        if (linMovTest(moved)) {
            return moved;
        }

        //moved = move(config, 0.001,0,0,0);
        if (linMovTest(moved)) {
            return moved;
        }

        return moved;
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
     * Expand ASV config one step
     * @param toExpand
     * @return
     */
    private List<ASVConfig> expand(ASVConfig toExpand) {
        List<ASVConfig> expanded = doExpansion(toExpand, -0.1);

        ASVConfig config = toExpand;

        List<ASVConfig> expansionToReturn = new ArrayList<>();


        int test = 100;
        while (!expansionTest(expanded.get(0)) && test>0) {
            config = linearTransformation(config);
            expansionToReturn.add(config);

            if (maximumExpansion(config)){
                ASVConfig rotated = rotate(config, 0.4);
                if (rotationTest(rotated)) {
                    config = rotated;
                    expansionToReturn.add(rotated);
                }
            }

            expanded = doExpansion(config, -0.1);

            test--;
        }
        expansionToReturn.addAll(expanded);
        return expansionToReturn;
    }

    /**
     * DO the actual expansion or contraction
     * @param toExpand
     * @param angle
     * @return
     */
    private List<ASVConfig> doExpansion(ASVConfig toExpand, double angle) {
        List<ASVConfig> expanded = new ArrayList<>();

        List<Point2D> positions = toExpand.getASVPositions();

        for (int i = 1; i < positions.size() - 1; i++) {
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
    private ASVConfig rotate(ASVConfig toRorate, double angle) {
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
        angle = angle * Math.PI/180;
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double x = ((toRotate.getX() - center.getX())*cos - (toRotate.getY() - center.getY())*sin) + center.getX();
        double y = ((toRotate.getX() - center.getX())*sin + (toRotate.getY() - center.getY())*cos) + center.getY();

        return new Point2D.Double(x,y);
    }


    /**
     * Returns if a point is same as another.
     * @param p1
     * @param p2
     * @return
     */
    private boolean samePoint(Point2D p1, Point2D p2) {
        return p1.distance(p2) <= 0.001;
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

            double distance = 0.05;

            double angle = Math.atan2(p1.getY() - p0.getY(), p1.getX() - p0.getX());
            double x = point.getX() + (Math.cos(angle)*distance);
            double y = point.getY() + (Math.sin(angle)*distance);
            PathPoint thisPoint = new PathPoint(x,y);
            point = thisPoint;
            transformed.add(thisPoint);
        }

        //System.out.println(expansionTest(new ASVConfig(transformed)));

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
        if (numberOfPoints <= test.DEFAULT_MAX_ERROR) {
            return nextPoint;
        }

        double x = p1.getX()+intervalX*0.001;
        double y = p1.getY()+intervalY*0.001;

        //System.out.println(p1.distance(new PathPoint(x,y)));

        return new PathPoint(x,y);
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
        boolean test2 = test.hasValidBoomLengths(next);
        boolean test3 = test.isConvex(next);
        boolean test4 = test.hasEnoughArea(next);
        boolean test5 = test.fitsBounds(next);

        return test1 && test2 && test3 && test4 && test5;
    }

    private boolean maximumExpansion(ASVConfig config) {
        boolean test3 = test.isConvex(config);
        boolean test4 = test.hasEnoughArea(config);

        return test3 && test4;
    }

}
