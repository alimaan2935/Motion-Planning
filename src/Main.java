
import problem.ProblemSpec;
import solution.OptimalConfigs;

import java.io.IOException;

public class Main {

    public static void main(String[] args) {

        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem(args[0]);
        }catch (IOException e) {

        }

        OptimalConfigs oc = new OptimalConfigs();
        oc.goalDistance(ps);
        try {
            ps.saveSolution(args[1]);
        }catch (IOException e) {

        }


    }
}
