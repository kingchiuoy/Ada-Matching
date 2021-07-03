package com.graphhopper.matching;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class Utils {
    public static void writeMappedPoints(List<double[]> points, int num) {
        try {
                FileWriter fileWriter = new FileWriter(new File("map-data/mapped-points" + String.valueOf(num) + ".csv"));
                for(double[] point : points) {
                    fileWriter.write(String.valueOf(point[0]));
                    fileWriter.write(",");
                    fileWriter.write(String.valueOf(point[1]));
                    fileWriter.write("\n");
                }
                fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void writeOriPoints(List<double[]> points, int num) {
        try {
            FileWriter fileWriter = new FileWriter(new File("map-data/ori-points" + String.valueOf(num) + ".csv"));
            for(double[] point : points) {
                fileWriter.write(String.valueOf(point[0]));
                fileWriter.write(",");
                fileWriter.write(String.valueOf(point[1]));
                fileWriter.write("\n");
            }
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static double getSpeedProb(double speed) {
        double prob;
        if(0.0 <= speed && speed < 20.0) {
            prob =  0.03;
        } else if(20.0 <= speed && 40 > speed) {
            prob = (-3.0 * speed) / 2000.0 + 0.06;
        } else {
            prob = 0.0;
        }
        return (prob * 90.0) / 3.0;
    }

    //速度评分函数
    public static double getAdjustSpeedProb(double speed, double v0) {
        double v_max = 2.0*v0;
        double c = Math.pow(v0+v_max,2.0)*3.0/(8.0*v0+4.0*v_max);
        double prob;
        if(0.0 <= speed && speed < v0) {
            prob =  2.0 / (v0+v_max);
        } else if(v0 <= speed && v_max >= speed) {
            prob = (2.0*v_max-2.0*speed)/(v_max*v_max-v0*v0);
        } else {
            prob = 0.0;
        }
        return prob*c;
    }


//    shunshizhen xiangshang wei
    public static double calDirection(double lat1, double lon1, double lat2, double lon2) {
        double y = Math.sin(lon2 - lon1) * Math.cos(lat2);
        double x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
        double theta = Math.atan2(y, x);
        return (theta * 180.0 / Math.PI + 360.0) % 360;
    }


    public static void longestPath(int start, int NODE, double[][] cost) {
        Stack<Integer> stk = new Stack<>();
        double dist[] = new double [NODE];
        boolean vis[] = new boolean[NODE];
        int next[] = new int[NODE];
        Arrays.fill(next, -1);

        for(int i = 0; i<NODE;i++)
            vis[i] = false;    // make all nodes as unvisited at first

        for(int i = 0; i<NODE; i++)
            dist[i] = Double.POSITIVE_INFINITY;    //initially all distances are infinity
        dist[start] = 0;    //distance for start vertex is 0

        while(!stk.empty()) {    //when stack contains element, process in topological order
            int nextVert = stk.peek(); stk.pop();
            if(dist[nextVert] != Double.POSITIVE_INFINITY) {
                for(int v = 0; v<NODE; v++) {
                    if(cost[nextVert][v] != Double.POSITIVE_INFINITY && cost[nextVert][v] != Double.POSITIVE_INFINITY) {
                        if(dist[v] < dist[nextVert] + cost[nextVert][v]) {
                            dist[v] = dist[nextVert] + cost[nextVert][v];
                            next[nextVert] = v;
                        }
                    }
                }
            }
        }

        int i = start;
        System.out.println(i);
        while(next[i]!=-1){//next数组初始化为-1
            i=next[i];
            System.out.println(i);
        }
    }

    static class longestGraph {

        int vertices;
        ArrayList<Integer> edge[];
        Map<Integer, Double> weight[];
        int next[];

        longestGraph(int vertices) {
            this.vertices = vertices;
            edge = new ArrayList[vertices + 1];
            weight = new HashMap[vertices + 1];
            for (int i = 0; i <= vertices; i++) {
                edge[i] = new ArrayList<>();
                weight[i] = new HashMap<>();
            }
            next = new int[vertices];
            Arrays.fill(next, -1);
        }

        void addEdge(int a, int b, double weights) {
            edge[a].add(b);
            weight[a].put(b, weights);
        }

        void dfs(int node, ArrayList<Integer> adj[], double dp[],
                 boolean visited[]) {
            // Mark as visited
            visited[node] = true;

            // Traverse for all its children
            for (int i = 0; i < adj[node].size(); i++) {

                // If not visited
                if (!visited[adj[node].get(i)])
                    dfs(adj[node].get(i), adj, dp, visited);

                // Store the max of the paths
                if (dp[adj[node].get(i)] + weight[node].get(adj[node].get(i)) > dp[node]) {
                    dp[node] = dp[adj[node].get(i)] + weight[node].get(adj[node].get(i));
                    next[node] = adj[node].get(i);
                }
            }
        }

        // Function that returns the longest path
        double findLongestPath(int n) {
            ArrayList<Integer> adj[] = edge;
            // Dp array
            double[] dp = new double[n + 1];

            // Visited array to know if the node
            // has been visited previously or not
            boolean[] visited = new boolean[n + 1];

            // Call DFS for every unvisited vertex
            for (int i = 1; i <= n; i++) {
                if (!visited[i])
                    dfs(i, adj, dp, visited);
            }

            double ans = 0d;

            // Traverse and find the maximum of all dp[i]
            for (int i = 1; i <= n; i++) {
                ans = Math.max(ans, dp[i]);
            }
            return ans;
        }


        int findLongestPathEnding(int n) {
            ArrayList<Integer> adj[] = edge;
            // Dp array
            double[] dp = new double[n + 1];

            // Visited array to know if the node
            // has been visited previously or not
            boolean[] visited = new boolean[n + 1];

            // Call DFS for every unvisited vertex
            for (int i = 1; i <= n; i++) {
                if (!visited[i])
                    dfs(i, adj, dp, visited);
            }

            double ans = 0d;
            int res = -1;


            // Traverse and find the maximum of all dp[i]
            for (int i = 1; i <= n; i++) {
                if(dp[i] > ans) {
                    ans = dp[i];
                    res = i;
                }
            }
            return res;
        }


        List<Integer> printPath(int i) {
            List<Integer> stateIds = new ArrayList<>();

            stateIds.add(i);
            while (next[i] != -1) {//next数组初始化为-1
                i = next[i];
                stateIds.add(i);
            }

            return stateIds;

        }
    }

    public static void main(String[] args) {
//        int n = 5;
//        longestGraph graph = new longestGraph(n);
//        // Example-1
//        graph.addEdge( 1, 2, 1);
//        graph.addEdge( 1, 3, 1);
//        graph.addEdge( 3, 2, 1);
//        graph.addEdge( 2, 4, 1);
//        graph.addEdge( 3, 4, 1);
//        graph.addEdge( 4, 2, 1);
//        int maxEnd = graph.findLongestPathEnding(n);

        System.out.println(4.9E-324 > 0.0);
//        System.out.println(graph.printPath(maxEnd));
//        System.out.println( graph.findLongestPath( n));

    }


}
