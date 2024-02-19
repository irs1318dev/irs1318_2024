package frc.lib.helpers;

import java.util.ArrayList;

public abstract class GraphNode
{
    final ArrayList<GraphLink> edgesFrom;

    protected GraphNode()
    {
        this.edgesFrom = new ArrayList<GraphLink>();
    }

    void addLink(GraphLink graphLink)
    {
        this.edgesFrom.add(graphLink);
    }
}
