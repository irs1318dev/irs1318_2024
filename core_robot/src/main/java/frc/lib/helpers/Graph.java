package frc.lib.helpers;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

public abstract class Graph<TGraphNode extends GraphNode>
{
    private final Set<TGraphNode> nodes;
    private final Map<TGraphNode, Map<TGraphNode, TGraphNode>> optimalPathMap;

    protected Graph()
    {
        this.nodes = new HashSet<TGraphNode>();
        this.optimalPathMap = new HashMap<TGraphNode, Map<TGraphNode, TGraphNode>>();
    }

    protected void addNode(TGraphNode node)
    {
        this.nodes.add(node);
    }

    public void connectBidirectional(TGraphNode node1, TGraphNode node2)
    {
        this.connectBidirectional(node1, node2, GraphLink.DEFAULT_WEIGHT);
    }

    public void connectBidirectional(TGraphNode node1, TGraphNode node2, double weight)
    {
        this.connectBidirectional(node1, node2, weight, weight);
    }

    public void connectBidirectional(TGraphNode node1, TGraphNode node2, double weight12, double weight21)
    {
        this.connect(node1, node2, weight12);
        this.connect(node2, node1, weight21);
    }

    public void connect(TGraphNode from, TGraphNode to)
    {
        this.connect(from, to, GraphLink.DEFAULT_WEIGHT);
    }

    public void connect(TGraphNode from, TGraphNode to, double weight)
    {
        from.addLink(new GraphLink(from, to, weight));
    }

    public Set<TGraphNode> getNodes()
    {
        return this.nodes;
    }

    public void precalculateOptimalPaths()
    {
        for (TGraphNode node : this.nodes)
        {
            this.optimalPathMap.put(node, this.dijkstra(node));
        }
    }

    public List<TGraphNode> getOptimalPath(TGraphNode start, TGraphNode end)
    {
        Map<TGraphNode, TGraphNode> optimalPreviousNodes = this.optimalPathMap.get(start);
        if (optimalPreviousNodes == null)
        {
            optimalPreviousNodes = this.dijkstra(start);
            this.optimalPathMap.put(start, optimalPreviousNodes);
        }

        LinkedList<TGraphNode> optimalPath = new LinkedList<TGraphNode>();
        TGraphNode node = end;
        while (node != null)
        {
            optimalPath.addFirst(node);
            node = optimalPreviousNodes.get(node);
        }

        if (optimalPath.getFirst() != start)
        {
            ExceptionHelpers.Assert(false, "The provided start node is not reachable from the provided end node.");
            return null;
        }

        return optimalPath;
    }

    // Java sucks and doesn't have actualized generics, so we have to suppress the unchecked warning
    @SuppressWarnings("unchecked")
    private Map<TGraphNode, TGraphNode> dijkstra(TGraphNode start)
    {
        // the optimal previous node along each potential path for each node based on the provided starting node
        Map<TGraphNode, TGraphNode> optimalPreviousNode = new HashMap<TGraphNode, TGraphNode>(this.nodes.size());

        // the current distance to each node from the starting node
        Map<GraphNode, Double> distanceMap = new HashMap<GraphNode, Double>(this.nodes.size());

        // the set of nodes that have not yet been visited
        HashSet<TGraphNode> unvisitedNodes = new HashSet<TGraphNode>(this.nodes.size());
        for (TGraphNode node : this.nodes)
        {
            if (node == start)
            {
                distanceMap.put(node, 0.0);
                unvisitedNodes.add(node);
            }
            else
            {
                distanceMap.put(node, Double.POSITIVE_INFINITY);
                unvisitedNodes.add(node);
            }
        }

        while (!unvisitedNodes.isEmpty())
        {
            TGraphNode node = null;
            for (TGraphNode unvisitedNode : unvisitedNodes)
            {
                if (node == null || distanceMap.get(unvisitedNode) < distanceMap.get(node))
                {
                    node = unvisitedNode;
                }
            }

            unvisitedNodes.remove(node);

            double nodeDistance = distanceMap.get(node);
            for (GraphLink link : node.edgesFrom)
            {
                GraphNode to = link.to;
                double distance = nodeDistance + link.weight;
                if (distance < distanceMap.get(to))
                {
                    distanceMap.put(to, distance);
                    optimalPreviousNode.put((TGraphNode)to, node);
                }
            }
        }

        return optimalPreviousNode;
    }
}
