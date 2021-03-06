package problem;

import java.util.*;

import java.awt.geom.Point2D;

/**
 * Represents a configuration of the ASVs. This class doesn't do any validity
 * checking - see the code in tester.Tester for this.
 *
 * @author lackofcheese
 */
public class ASVConfig {
	/** The position of each ASV */
	private List<Point2D> asvPositions = new ArrayList<Point2D>();

	protected ASVConfig parent;
	protected double distanceFromStart = Integer.MAX_VALUE;
	protected double distanceToGoal;
	protected boolean hasParent = false;
	protected Set<ASVConfig>  children;
	protected double[] angles;
	private boolean visited = false;

	/**
	 * Constructor. Takes an array of 2n x and y coordinates, where n is the
	 * number of ASVs
	 *
	 * @param coords
	 *            the x- and y-coordinates of the ASVs.
	 */
	public ASVConfig(double[] coords) {
		for (int i = 0; i < coords.length / 2; i++) {
			asvPositions.add(new Point2D.Double(coords[i * 2],
					coords[i * 2 + 1]));
		}
		this.children = new HashSet<>();
	}

	/**
	 * Constructor. Takes a List of Point2D objects
	 *
	 * @param coords
	 *            Point2d coordinates of the ASVs.
	 */
	public ASVConfig(List<Point2D> coords) {
		for (int i = 0; i < coords.size(); i++) {
			asvPositions.add(coords.get(i));
		}
		this.children = new HashSet<>();
	}

	/**
	 * Constructs an ASVConfig from a space-separated string of x- and y-
	 * coordinates
	 *
	 * @param asvCount
	 *            the number of ASVs to read.
	 * @param str
	 *            the String containing the coordinates.
	 */
	public ASVConfig(int asvCount, String str) throws InputMismatchException {
		Scanner s = new Scanner(str);
		for (int i = 0; i < asvCount; i++) {
			asvPositions
					.add(new Point2D.Double(s.nextDouble(), s.nextDouble()));
		}
		s.close();
		this.children = new HashSet<>();
	}

	/**
	 * Copy constructor.
	 *
	 * @param cfg
	 *            the configuration to copy.
	 */
	public ASVConfig(ASVConfig cfg) {
		asvPositions = cfg.getASVPositions();
		this.children = new HashSet<>();
	}

	/**
	 * Returns a space-separated string of the ASV coordinates.
	 *
	 * @return a space-separated string of the ASV coordinates.
	 */
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for (Point2D point : asvPositions) {
			if (sb.length() > 0) {
				sb.append(" ");
			}
			sb.append(point.getX());
			sb.append(" ");
			sb.append(point.getY());
		}
		return sb.toString();
	}

	/**
	 * Returns the maximum straight-line distance between the ASVs in this state
	 * vs. the other state, or -1 if the ASV counts don't match.
	 *
	 * @param otherState
	 *            the other state to compare.
	 * @return the maximum straight-line distance for any ASV.
	 */
	public double maxDistance(ASVConfig otherState) {
		if (this.getASVCount() != otherState.getASVCount()) {
			return -1;
		}
		double maxDistance = 0;
		for (int i = 0; i < this.getASVCount(); i++) {
			double distance = this.getPosition(i).distance(
					otherState.getPosition(i));
			if (distance > maxDistance) {
				maxDistance = distance;
			}
		}
		return maxDistance;
	}

	/**
	 * Returns the total straight-line distance over all the ASVs between this
	 * state and the other state, or -1 if the ASV counts don't match.
	 *
	 * @param otherState
	 *            the other state to compare.
	 * @return the total straight-line distance over all ASVs.
	 */
	public double totalDistance(ASVConfig otherState) {
		if (this.getASVCount() != otherState.getASVCount()) {
			return -1;
		}
		double totalDistance = 0;
		for (int i = 0; i < this.getASVCount(); i++) {
			totalDistance += this.getPosition(i).distance(
					otherState.getPosition(i));
		}
		return totalDistance;
	}

	/**
	 * Returns the position of the ASV with the given number.
	 *
	 * @param asvNo
	 *            the number of the ASV.
	 * @return the position of the ASV with the given number.
	 */
	public Point2D getPosition(int asvNo) {
		return asvPositions.get(asvNo);
	}

	/**
	 * Returns the number of ASVs in this configuration.
	 *
	 * @return the number of ASVs in this configuration.
	 */
	public int getASVCount() {
		return asvPositions.size();
	}

	/**
	 * Returns the positions of all the ASVs, in order.
	 *
	 * @return the positions of all the ASVs, in order.
	 */
	public List<Point2D> getASVPositions() {
		return new ArrayList<Point2D>(asvPositions);
	}

	public ASVConfig getParent() {
		return parent;
	}

	public void setParent(ASVConfig parent) {
		this.parent = parent;
		hasParent = true;
	}

	public double getDistanceFromStart() {
		return distanceFromStart;
	}

	public void setDistanceFromStart(double distanceFromStart) {
		this.distanceFromStart = distanceFromStart;
	}

	public double getDistanceToGoal() {
		return distanceToGoal;
	}

	public void setDistanceToGoal(double distanceToGoal) {
		this.distanceToGoal = distanceToGoal;
	}

	public boolean hasParent() {
		return hasParent;
	}

	public double getF() {
		return distanceFromStart + distanceToGoal;
	}


	public int getConvexityDirection() {
		Point2D first = this.asvPositions.get(0);
		Point2D second = this.asvPositions.get(1);
		Point2D last = this.asvPositions.get(asvPositions.size()-1);

		double angleSecond = Math.atan2(second.getY() - first.getY(), second.getX() - first.getX());
		double angleLast = Math.atan2(last.getY() - first.getY(), last.getX() - first.getX());

		double direction = angleSecond-angleLast;

		if (direction < 0) {
			return -1;
		}else {
			return 1;
		}

	}

	/**
	 * returns config internal angles
	 * @return
	 */
	public double[] getAngles() {
		this.angles = calculateAngles(asvPositions);
		return angles;
	}

	/**
	 * Calculate internal angles of given ASV configuration
	 * @param ASVPositions
	 * @return
	 */
	private double[] calculateAngles(List<Point2D> ASVPositions) {
		double[] angles = new double[ASVPositions.size()];

		angles[0] = 0;

		for (int i = 1; i< angles.length; i++) {
			Point2D p0 = ASVPositions.get(i-1);
			Point2D p1 = ASVPositions.get(i);
			angles[i] = Math.atan2(p1.getY() - p0.getY(), p1.getX() - p0.getX());
		}

		return angles;
	}

	/**
	 * Returns the area of config.

	 * @return whether the given configuration has sufficient area.
	 */
	public double getArea() {
		double total = 0;
		List<Point2D> points = getASVPositions();
		points.add(points.get(0));
		points.add(points.get(1));
		for (int i = 1; i < points.size() - 1; i++) {
			total += points.get(i).getX()
					* (points.get(i + 1).getY() - points.get(i - 1).getY());
		}
		double area = Math.abs(total) / 2;
		return area;
	}

	public Set<ASVConfig> getChildren() {
		return children;
	}

	public void addChild(ASVConfig children) {
		this.children.add(children);
	}

	public void setVisited(boolean visited) {
		this.visited = visited;
	}

	public boolean isVisited() {
		return visited;
	}
}
