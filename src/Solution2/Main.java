package Solution2;

import problem.ProblemSpec;
import solution.Motion;
import solution.ValidPoints;
import tester.Tester;

import java.io.IOException;

public class Main {


    public static void main(String[] args) {

        // args = new String[] {"testcases/3ASV-easy.txt", "testcases/exp.txt", "-v"};
        // args = new String[] {"testcases/3ASV-x4.txt", "testcases/exp.txt", "-v"};
        args = new String[] {"testcases/3ASV.txt", "testcases/exp.txt", "-v"};
        // args = new String[] {"testcases/7ASV-easy.txt", "testcases/exp.txt", "-v"};
        // args = new String[] {"testcases/7-ASV-x6.txt", "testcases/exp.txt", "-v"};
        // args = new String[] {"testcases/7-ASV-x4.txt", "testcases/exp.txt", "-v"};
        // args = new String[] {"testcases/7-ASV-x2.txt", "testcases/exp.txt", "-v"};
        // args = new String[] {"testcases/7ASV.txt", "testcases/exp.txt", "-v"};
        // args = new String[] {"testcases/p.txt", "testcases/exp.txt", "-v"};

        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem(args[0]);
        }catch (IOException e) {

        }

        long start = System.currentTimeMillis();



        MotionPlanner motionPlanner = new MotionPlanner(ps.getInitialState(), ps.getGoalState(), ps, new Tester());


        ps.setPath(motionPlanner.getPath());

        long end = System.currentTimeMillis();

        System.out.println("Time Taken: " + (end-start));


        try {
            ps.saveSolution(args[1]);
        }catch (IOException e) {

        }

        Tester tester = new Tester();
        tester.main(args);


    }
}
