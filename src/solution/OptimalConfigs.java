package solution;

import problem.ASVConfig;
import problem.ProblemSpec;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class OptimalConfigs {

    ProblemSpec ps;
    OptimalPoints op;


    public OptimalConfigs (ProblemSpec ps) {
        this.ps = ps;
        op = new OptimalPoints(ps);
    }


    /**
     * set the path
     */
    public void setPath () {
        ps.setPath(genASVConfigs());
    }

    /**
     * Generate All for initial and goal ASV configs
     */
    public ArrayList<ASVConfig> genASVConfigs() {

        int ASVCount = ps.getASVCount();
        ArrayList<ArrayList<Point2D>> allConfigs = new ArrayList<>();
        ArrayList<ASVConfig> configs = new ArrayList<>();

        for (int i = 0; i<ASVCount; i++) {
            allConfigs.add(op.getPoints(ps.getInitialState().getASVPositions().get(i), ps.getGoalState().getASVPositions().get(i)));
        }

        int steps = 0;
        for (int i =0; i< allConfigs.size(); i++) {
            if (steps < allConfigs.get(i).size()) {
                steps = allConfigs.get(i).size();
            }
        }

        return getConfigs(allConfigs, steps);
    }

    public ArrayList<ASVConfig> getConfigs(ArrayList<ArrayList<Point2D>> allConfigs, int steps) {
        ArrayList<ASVConfig> ASVConfigs = new ArrayList<>();

        Point2D[] lastPoints = new Point2D[allConfigs.size()];

        for (int i =0; i< steps; i++) {
            double[] coord = new double[allConfigs.size()*2];
            for (int j=0; j<allConfigs.size(); j++) {

                if (i >= allConfigs.get(j).size()) {
                    coord[j*2] = lastPoints[j].getX();
                    coord[j*2+1] = lastPoints[j].getY();
                }else {
                    Point2D point = allConfigs.get(j).get(i);
                    coord[j * 2] = point.getX();
                    coord[j * 2 + 1] = point.getY();
                    lastPoints[j] = point;
                }
            }
            ASVConfig c = new ASVConfig(coord);
            ASVConfigs.add(c);
        }

        return ASVConfigs;
    }
}
