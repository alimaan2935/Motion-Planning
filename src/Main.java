
import problem.ProblemSpec;
import solution.OptimalPoints;

import java.io.IOException;

public class Main {

    public static void main(String[] args) {

        args = new String[] {"testcases/3ASV-easy.txt", "testcases/exp.txt"};

        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem(args[0]);
        }catch (IOException e) {

        }

        OptimalPoints op = new OptimalPoints(ps);
        System.out.println(op.getPoints());

        try {
            ps.saveSolution(args[1]);
        }catch (IOException e) {

        }


    }
}
