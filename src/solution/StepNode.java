package solution;

import problem.ASVConfig;


public class StepNode {

    protected ASVConfig config;
    protected StepNode parent;
    protected double distanceFromStart;
    protected double distanceToGoal;
    protected boolean hasParent = false;

    public StepNode (ASVConfig config) {
        this.config = config;
    }

    public ASVConfig getConfig() {
        return config;
    }

    public StepNode getParent() {
        return parent;
    }

    public void setParent(StepNode parent) {
        this.parent = parent;
        hasParent = true;
    }

    public boolean hasParent() {
        return hasParent;
    }

    public double getF() {
        return distanceFromStart + distanceToGoal;
    }

    public void setDistanceFromStart(double distanceFromStart) {
        this.distanceFromStart = distanceFromStart;
    }

    public void setDistanceToGoal(double distanceToGoal) {
        this.distanceToGoal = distanceToGoal;
    }

    public double getDistanceFromStart() {
        return distanceFromStart;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }
}
