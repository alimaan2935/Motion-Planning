package Solution2;

import problem.ASVConfig;
import problem.ProblemSpec;
import solution.PathPoint;
import tester.Tester;

import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.*;

public class Graph {

    ASVConfig startConfig;
    ASVConfig goalConfig;
    List<ASVConfig> randomConfigs;
    LocalPlanner localPlanner;
    Tester tester;
    ProblemSpec ps;


    public Graph(ASVConfig startConfig, ASVConfig goalConfig, List<ASVConfig> randomConfigs, LocalPlanner localPlanner, ProblemSpec ps, Tester tester) {
        this.startConfig = startConfig;
        this.goalConfig = goalConfig;
        this.randomConfigs = randomConfigs;
        this.localPlanner = localPlanner;
        this.tester = tester;
        this.ps = ps;
    }

    /**
     * Get optimal path configs
     * @return
     */
    public List<ASVConfig> getPath() {
        List<ASVConfig> pathConfigs = new ArrayList<>();

        generateGraph();

        List<ASVConfig> optimalPathConfigs = graphSearch();

        pathConfigs.addAll(giveMeConfigs(optimalPathConfigs));

        return pathConfigs;
    }

    /**
     * The final moment for these robots to be connected.
     * They should be happy now.
     * @param pathConfigs
     * @return
     */
    private List<ASVConfig> giveMeConfigs(List<ASVConfig> pathConfigs) {
        List<ASVConfig> configsToReturn = new ArrayList<>();

        for (int i = 0; i<pathConfigs.size()-1; i++) {
            configsToReturn.addAll(goForIt(pathConfigs.get(i), pathConfigs.get(i+1)));
        }
        return configsToReturn;
    }

    /**
     * Please get solved this time.
     * @param config1
     * @param config2
     * @return
     */
    private List<ASVConfig> goForIt(ASVConfig config1, ASVConfig config2) {
        List<ASVConfig> configsToReturn = new ArrayList<>();
        List<Point2D> pointsOnPath = localPlanner.allPointsOnPath(config1.getASVPositions().get(0), config2.getASVPositions().get(0));
        int number = pointsOnPath.size();
        double[] differenceAngleParts = getDiffAngleParts(config1, config2, number);

        ASVConfig currentConfig = config1;
        for (int i = 0; i < number; i++) {
            ASVConfig theNextOne = generateConfig(pointsOnPath.get(i), currentConfig, differenceAngleParts);
            configsToReturn.add(theNextOne);
            currentConfig = theNextOne;

        }
        return configsToReturn;
    }
/*
    private List<ASVConfig> goForIt(ASVConfig config1, ASVConfig config2) {
        List<ASVConfig> configsToReturn = new ArrayList<>();

        ASVConfig start = config1;

        int test = 200;
        while (!localPlanner.sameConfig(start, config2) && test>0) {
            ASVConfig nextStep = localPlanner.nextStep(start, config2);
            if (tester.validConfig(nextStep, ps)) {
                start = nextStep;
                configsToReturn.add(nextStep);
            }else {
                start = doValidRotation(start);
                configsToReturn.add(start);
            }
            test--;
        }
        return configsToReturn;
    }*/

    private ASVConfig doValidRotation(ASVConfig config) {

        ASVConfig rotated1 = localPlanner.rotate(config, Tester.DEFAULT_EXP_ANGLE);
        ASVConfig rotated2 = localPlanner.rotate(config, -Tester.DEFAULT_EXP_ANGLE);

        if (tester.validConfig(rotated1, ps)) {
            return rotated1;
        }else if (tester.validConfig(rotated2, ps)) {
            return rotated2;
        }
        return config;
    }

    private double[] getDiffAngleParts(ASVConfig config1, ASVConfig config2, int number) {
        double[] diff = new double[config1.getAngles().length];

        for (int i = 0; i < diff.length; i++) {

            diff[i] = (toDegrees(config1.getAngles()[i]) - toDegrees(config2.getAngles()[i])) / number;
        }
        return diff;
    }



    /**
     * Generate a configuration based on first point and all the angles
     * @param point
     * @param previous
     * @return
     */
    private ASVConfig generateConfig(Point2D point, ASVConfig previous,  double[] diff) {
        List<Point2D> configPoints = new ArrayList<>();
        configPoints.add(point);

        double[] angles = previous.getAngles();
        for (int i = 1; i< angles.length; i++) {
            double a = toRadians(toDegrees(angles[i]) - diff[i]);
            angles[i] = a;
        }

        for (int i = 1; i < angles.length; i++) {
            double x = configPoints.get(configPoints.size()-1).getX() + (Math.cos(angles[i])* Tester.MAX_BOOM_LENGTH);
            double y = configPoints.get(configPoints.size()-1).getY() + (Math.sin(angles[i])*Tester.MAX_BOOM_LENGTH);
            PathPoint thisPoint = new PathPoint(x,y);
            configPoints.add(thisPoint);
        }
        return new ASVConfig(configPoints);
    }


    /**
     * Connect all of the configs with their children such that parent has a collision free path to all children
     */
    private void generateGraph() {
        randomConfigs.add(startConfig);
        randomConfigs.add(goalConfig);
        for (int i = 0; i < randomConfigs.size(); i++) {
            ASVConfig config1 = randomConfigs.get(i);
            for (int j = 0; j < randomConfigs.size(); j++) {
                ASVConfig config2 = randomConfigs.get(j);
                if (localPlanner.freePathExist(config1,config2) && validateAngles(config1, config2)) {
                    config1.addChild(config2);
                }
            }
            int bp = 0;
        }
    }


    /**
     * Generate best path
     * @return
     */
    private List<ASVConfig> graphSearch() {
        List<ASVConfig> searchedConfigs = new ArrayList<>();
        searchedConfigs.add(startConfig);

        ASVConfig currentExpanding = startConfig;
        double distance = Integer.MAX_VALUE;


        List<ASVConfig> bestChilds = new ArrayList<>();

        int test = 1000000;
        while (!localPlanner.sameConfig(currentExpanding,goalConfig) && test>0) {
            ASVConfig bestChild = currentExpanding;
            for (ASVConfig c : currentExpanding.getChildren()) {
                double d = c.maxDistance(goalConfig);

                if (distance > d && !bestChilds.contains(c)) {
                    distance = d;
                    bestChild = c;
                    bestChilds.add(c);
                }
            }
            currentExpanding = bestChild;
            searchedConfigs.add(bestChild);
            distance = Integer.MAX_VALUE;
            test--;
        }

        return searchedConfigs;
    }
/*
    private List<ASVConfig> graphSearch() {
        List<ASVConfig> searchedConfigs = new ArrayList<>();
        searchedConfigs.add(startConfig);

        ASVConfig currentExpanding = startConfig;
        currentExpanding.setDistanceFromStart(0);


        int test = 10000;
        while (!localPlanner.sameConfig(currentExpanding,goalConfig) && test>0) {
            for (ASVConfig c : currentExpanding.getChildren()) {
                double newDistance = currentExpanding.getDistanceFromStart() + c.maxDistance(currentExpanding);

                if (newDistance < c.getDistanceFromStart()) {
                    c.setDistanceFromStart(newDistance);
                    if (!c.isVisited()) {
                        c.setVisited(true);
                        searchedConfigs.add(c);
                    }
                }
            }
            currentExpanding = searchedConfigs.get(searchedConfigs.size()-1);
            test--;
        }

        return searchedConfigs;
    }

/*
    private List<ASVConfig> graphSearch(){

        startConfig.setDistanceFromStart(0);

        //Comparator and priority queue
        MinHeuristicsComparator comparator = new MinHeuristicsComparator();
        //comparator.setGoal(goalConfig);
        PriorityQueue<ASVConfig> queue = new PriorityQueue<>(comparator);

        queue.add(startConfig);

        while (!queue.isEmpty()) {
            ASVConfig currentNode = queue.poll();

            Set<ASVConfig> children = currentNode.getChildren();

            for (ASVConfig c : children) {
                c.setParent(currentNode);

                double newDistance = currentNode.getDistanceFromStart() + c.maxDistance(currentNode);


                if (localPlanner.sameConfig(c, goalConfig)) {
                    break;
                }

                if (newDistance < c.getDistanceFromStart()) {
                    c.setDistanceFromStart(newDistance);
                    if (!c.isVisited()) {
                        queue.add(c);
                    }
                }
            }
        }

        Stack<ASVConfig> pathStack = new Stack<>();
        ASVConfig cur = goalConfig;
        pathStack.push(cur);
        while (cur.hasParent()) {
            pathStack.push(cur.getParent());
            cur = cur.getParent();
        }

        List<ASVConfig> configsToReturn = new ArrayList<>();

        while (!pathStack.isEmpty()) {
            configsToReturn.add(pathStack.pop());
        }

        return configsToReturn;

    }
*/

    private double whatDistance (ASVConfig c, ASVConfig goalConfig) {
        if (localPlanner.freePathExist(c, goalConfig)) {
            return c.maxDistance(goalConfig);
        }
        return (c.maxDistance(goalConfig))*2;
    }


    private double toDegrees(double radians) {
        radians = Math.toDegrees(radians);
        if (radians < 0) {
            radians += 360;
        }
        return radians;
    }

    private double toRadians(double degrees) {
        if (degrees > 180) {
            degrees -= 360;
        }
        return Math.toRadians(degrees);
    }

    private boolean validateAngles(ASVConfig c1, ASVConfig c2) {

        double diff = Integer.MIN_VALUE;

        for (int i = 0 ; i < c1.getAngles().length; i++) {
            double a1 = toDegrees(c1.getAngles()[i]);
            double a2 = toDegrees(c2.getAngles()[i]);

            double ldiff = Math.abs(a2-a1);
            if (ldiff >= diff) {
                diff = ldiff;
            }
        }

        return diff < 50;

    }

    public class MinHeuristicsComparator implements Comparator<ASVConfig> {

        ASVConfig goal;

        public void setGoal(ASVConfig goal) {
            this.goal = goal;
        }


        @Override
        public int compare(ASVConfig c1, ASVConfig c2) {

            return Double.compare(c1.getDistanceFromStart(), c2.getDistanceFromStart());
        }

    }
}
