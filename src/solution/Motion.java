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

        ASVConfig start = startConfig;


        int test = 2;
        while (!path.isEmpty()) {
            // MotionSearch motionSearch = new MotionSearch(start, path.pop(), ps, new Tester());
            // MotionPlan motionPlan = new MotionPlan(start, path.pop(), ps);

            RealMotionPlan realMotionPlan = new RealMotionPlan(start, path.pop(), ps, startConfig.getConvexityDirection());

            completePath.addAll(realMotionPlan.getPathConfigs());
            start = completePath.get(completePath.size()-1);

            test--;
        }

        completePath.add(goalConfig);


        return completePath;
    }


}
