package solution;


import problem.ProblemSpec;
import tester.Tester;

import java.io.IOException;

public class Testing {




    public static void main(String[] args) {

        // args = new String[] {"testcases/3ASV-easy.txt", "testcases/exp.txt", "-v"};
        // args = new String[] {"testcases/3ASV-x4.txt", "testcases/exp.txt", "-v"};
        // args = new String[] {"testcases/3ASV.txt", "testcases/exp.txt", "-v"};
        args = new String[] {"testcases/7ASV-easy.txt", "testcases/exp.txt", "-v"};
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

        ValidPoints v = new ValidPoints(ps);
        Motion motion = new Motion(ps, v.getPath());

        long start = System.currentTimeMillis();

        ps.setPath(motion.getCompletePath());

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

/**
 * Conclusion:
 * Move one step towards next point - check if any point is colliding with obstacle
 * if not: move to the next point
 * if collides, transform that point from previous point i.e: center point is the previous point and moving points are the point
 * of collision and next point
 */
