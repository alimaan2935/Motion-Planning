
import problem.ASVConfig;
import problem.ProblemSpec;
import solution.*;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.*;

public class Main {



    public static void main(String[] args) {

        args = new String[] {"testcases/7ASV.txt", "testcases/exp.txt"};

        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem(args[0]);
        }catch (IOException e) {

        }


        ValidPoints points = new ValidPoints(ps);
        points.getPath();


        try {
            ps.saveSolution(args[1]);
        }catch (IOException e) {

        }



    }
}
