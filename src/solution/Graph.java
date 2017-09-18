package solution;

import problem.ProblemSpec;

public class Graph {

    ProblemSpec ps;
    ValidPoints vp;

    public Graph (ProblemSpec ps) {
        this.ps = ps;
        this.vp = new ValidPoints(ps);
    }

}
