package Solution2;

import com.sun.tools.javah.Gen;
import problem.ASVConfig;
import problem.ProblemSpec;
import tester.Tester;

import java.util.ArrayList;
import java.util.List;

/**
 * Main Driver method for motion planning
 */
public class MotionPlanner {

    ASVConfig startConfig;
    ASVConfig goalConfig;
    ASVConfig maxExpandedConfig;

    ProblemSpec ps;
    Tester tester;


    public MotionPlanner (ASVConfig startConfig, ASVConfig goalConfig, ProblemSpec ps, Tester tester) {
        this.startConfig = startConfig;
        this.goalConfig = goalConfig;
        this.ps = ps;
        this.tester = tester;
    }

    public List<ASVConfig> getPath() {
        List<ASVConfig> pathToReturn = new ArrayList<>();
        pathToReturn.add(startConfig);

        LocalPlanner localPlanner = new LocalPlanner(startConfig, goalConfig, ps, tester);

        if (localPlanner.freePathExist(startConfig, goalConfig)) {
            pathToReturn.addAll(localPlanner.getFreePath());
            return pathToReturn;
        }

        pathToReturn.addAll(localPlanner.getMaximumExpansion());

        GenerateConfigs configs = new GenerateConfigs(pathToReturn.get(pathToReturn.size()-1), goalConfig.getASVPositions().get(0), ps, tester);
        List<ASVConfig> randomConfigs = configs.getRandomConfigs();
        startConfig = pathToReturn.get(pathToReturn.size()-1);

        Graph graph = new Graph(startConfig, goalConfig, randomConfigs, localPlanner, ps, tester);

        pathToReturn.addAll(graph.getPath());





        return pathToReturn;
    }

}
