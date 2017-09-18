package solution;


import problem.ProblemSpec;
import tester.Tester;

import java.io.IOException;

public class Testing {




    public static void main(String[] args) {

        args = new String[] {"testcases/7-ASV-x6.txt", "testcases/exp.txt", "-v"};

        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem(args[0]);
        }catch (IOException e) {

        }

        ValidPoints v = new ValidPoints(ps);
        //CompletePath completePath = new CompletePath(ps, v.getPath());
        NewSolution newSolution = new NewSolution(ps, v.getPath());
        ps.setPath(newSolution.getCompletePath());


        try {
            ps.saveSolution(args[1]);
        }catch (IOException e) {

        }

        Tester tester = new Tester();
        tester.main(args);


    }
}

/**
 * Conclusion:
 * Move one step towards next point - check if any point is colliding with obstacle
 * if not: move to the next point
 * if collides, transform that point from previous point i.e: center point is the previous point and moving points are the point
 * of collision and next point
 */
