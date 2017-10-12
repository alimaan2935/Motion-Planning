package Solution2;

import org.w3c.dom.css.Rect;
import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import solution.PathPoint;
import tester.Tester;

import java.util.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

/**
 * Generate random configurations in an area similar to the provided config.
 */
public class GenerateConfigs {

    ASVConfig startConfig;
    Point2D goalPoint;
    ProblemSpec ps;
    Tester tester;

    public GenerateConfigs (ASVConfig startConfig, Point2D goalPoint, ProblemSpec ps, Tester tester){
        this.startConfig = startConfig;
        this.goalPoint = goalPoint;
        this.ps = ps;
        this.tester = tester;
    }

    /**
     * Generates random configs and returns the list
     * @return
     */
    public List<ASVConfig> getRandomConfigs() {
        List<ASVConfig> configsToReturn = new ArrayList<>();

        List<Rectangle2D> randomConfigBounds = getMaxObstacleBounds();

        List<Point2D> allPoints = generateRandomPointsInBounds(randomConfigBounds);

        configsToReturn.addAll(generateConfigsOnPoints(allPoints));


        return configsToReturn;
    }

    /**
     * Genertaes many configs on list of points
     * @param points
     * @return
     */
    private List<ASVConfig> generateConfigsOnPoints(List<Point2D> points) {
        List<ASVConfig> configsToReturn = new ArrayList<>();

        for (int i = 0; i < points.size(); i++) {
            configsToReturn.addAll(generateConfigsOnOnePoint(points.get(i)));
        }

        return configsToReturn;
    }

    /**
     * Generates a number of configs on a single point
     * @param point
     * @return
     */
    private List<ASVConfig> generateConfigsOnOnePoint(Point2D point) {
        List<ASVConfig> list = new ArrayList<>();
        int counter = 3;

        int tries = 100;
        while (counter>0 && tries>0) {
            ASVConfig config = generateRandomConfig(point);
            if (tester.validConfig(config, ps)) {
                list.add(config);
                counter--;
            }
            tries--;
        }
        return list;
    }

    /**
     * Generate a random configuration on a point
     * @param point
     * @return
     */
    private ASVConfig generateRandomConfig(Point2D point) {
        double[] angles = startConfig.getAngles();
        double differential = genmerateRandomNumber();

        for (int i = 1; i<angles.length; i++) {
            angles[i] += differential;
        }
        return generateConfig(point, angles);
    }

    /**
     * generate a random number between pi and negative pi
     * @return
     */
    private double genmerateRandomNumber() {
        Random r = new Random();
        double min = -Math.PI;
        double max = Math.PI;

        double ran = min + r.nextDouble() * (max - min);
        return ran;
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
     * Generate random points within the rectangular bounds but not in obstacles
     * @param obstacles
     * @return
     */
    private List<Point2D> generateRandomPointsInBounds(List<Rectangle2D> obstacles) {
        List<Point2D> points = new ArrayList<>();
        int counter = 0;

        List<Obstacle> orr = ps.getObstacles();
        List<Rectangle2D> grew = obstacles;

        Random r = new Random();
        double min = 0.01;
        double max = 0.99;

        while (counter<500) {
            double x = min + r.nextDouble() * (max - min);
            double y = min + r.nextDouble() * (max - min);
            Point2D p = new Point2D.Double(x,y);

            for (int i = 0; i< grew.size(); i++) {
                if (grew.get(i).contains(p)) {
                    Rectangle2D re = grow(orr.get(i).getRect(), Tester.MAX_STEP);
                    if (!re.contains(p)) {
                        points.add(p);
                        counter++;
                    }
                }
            }
        }
        return points;
    }


    /**
     * Returns max obstacle bounds to generate random configs.
     * @return
     */
    private List<Rectangle2D> getMaxObstacleBounds() {
        List<Rectangle2D> boundsToReturn = new ArrayList<>();

        List<Obstacle> obstacles = ps.getObstacles();
        for (int i = 0; i < obstacles.size(); i++) {
            boundsToReturn.add(grow(obstacles.get(i).getRect(), startConfig.getArea() *3));
        }
        return boundsToReturn;
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
        return new Rectangle2D.Double(rect.getX() - delta, rect.getY() - delta,
                rect.getWidth() + delta * 2, rect.getHeight() + delta * 3);
    }





}
