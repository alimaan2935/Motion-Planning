package Solution2;

import problem.ASVConfig;

import java.util.Comparator;

public class MinHeuristicsComparator implements Comparator<ASVConfig> {


    @Override
    public int compare(ASVConfig c1, ASVConfig c2) {

        return Double.compare(c1.getF(), c2.getF());
    }

}
