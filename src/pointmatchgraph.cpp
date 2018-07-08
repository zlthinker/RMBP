#include "pointmatchgraph.h"
#include <cassert>


void PointMatchGraph::addNode(size_t node_index, NodeT const & match_node)
{
    LemonNode const & node = lemon_graph_.addNode();
    std::tr1::unordered_map<size_t, LemonNode>::const_iterator node_itr = index_node_map_.find(node_index);
    assert(node_itr == index_node_map_.end() && "[PointMatchGraph::addNode] Add existed node.");
    index_node_map_[node_index] = node;
    node_map_[node] = match_node;
}

bool PointMatchGraph::nodeExist(size_t node_index) const
{
    return index_node_map_.find(node_index) != index_node_map_.end();
}

PointMatchGraph::NodeT & PointMatchGraph::node(size_t node_index)
{
    std::tr1::unordered_map<size_t, LemonNode>::const_iterator node_itr = index_node_map_.find(node_index);
    assert(node_itr != index_node_map_.end() && "[PointMatchGraph::node] Node does not exist.");
    LemonNode lemon_node = node_itr->second;
    return node_map_[lemon_node];
}

PointMatchGraph::NodeT const & PointMatchGraph::node(size_t node_index) const
{
    std::tr1::unordered_map<size_t, LemonNode>::const_iterator node_itr = index_node_map_.find(node_index);
    assert(node_itr != index_node_map_.end() && "[PointMatchGraph::node] Node does not exist.");
    LemonNode lemon_node = node_itr->second;
    return node_map_[lemon_node];
}

void PointMatchGraph::nodeIDs(std::vector<size_t> & node_indexes) const
{
    node_indexes.reserve(index_node_map_.size());
    for(std::tr1::unordered_map<size_t, LemonNode>::const_iterator node_itr = index_node_map_.begin()
        ; node_itr != index_node_map_.end(); node_itr++)
    {
        node_indexes.push_back(node_itr->first);
    }

    std::sort(node_indexes.begin(), node_indexes.end());
}

int PointMatchGraph::nodeDegree(size_t node_index) const
{
    std::tr1::unordered_map<size_t, LemonNode>::const_iterator node_itr = index_node_map_.find(node_index);
    assert(node_itr != index_node_map_.end() && "[PointMatchGraph::nodeDegree] Node does not exist.");
    LemonNode const & node = node_itr->second;
    int node_degree = 0;
    for(LemonGraph::OutArcIt edge_itr(lemon_graph_, node); edge_itr != lemon::INVALID; ++edge_itr)
    {
        node_degree++;
    }

    return node_degree;
}
void PointMatchGraph::adjacentNodeIDs(size_t node_index, std::vector<size_t> & node_indexes) const
{
    std::tr1::unordered_map<size_t, LemonNode>::const_iterator node_itr = index_node_map_.find(node_index);
    assert(node_itr != index_node_map_.end() && "[PointMatchGraph::adjacentNodeIDs] Node does not exist.");
    LemonNode const & node = node_itr->second;
    for(LemonGraph::OutArcIt edge_itr(lemon_graph_, node); edge_itr != lemon::INVALID; ++edge_itr)
    {
        LemonEdge edge(edge_itr);
        LemonNode opp_node(lemon_graph_.oppositeNode(node, edge) );
        NodeT const & camera_node = node_map_[opp_node];
        node_indexes.push_back(camera_node.index());
    }
    std::sort(node_indexes.begin(), node_indexes.end());
}

size_t PointMatchGraph::edgeNum() const
{
    std::vector<std::pair<size_t, size_t> > pairs;
    for(std::tr1::unordered_map<size_t, std::tr1::unordered_map<size_t, LemonEdge> >::const_iterator node_itr1 = index_edge_map_.begin()
        ; node_itr1 != index_edge_map_.end(); node_itr1++)
    {
        size_t node_index1 = node_itr1->first;
        std::tr1::unordered_map<size_t, LemonEdge> const & edge_map = node_itr1->second;
        for(std::tr1::unordered_map<size_t, LemonEdge>::const_iterator node_itr2 = edge_map.begin()
            ; node_itr2 != edge_map.end(); node_itr2++)
        {
            size_t node_index2 = node_itr2->first;
            pairs.push_back(std::make_pair(node_index1, node_index2));
        }
    }

    size_t pair_num = pairs.size();
    assert(pair_num % 2 == 0);
    return pair_num / 2;
}

void PointMatchGraph::addEdge(size_t node_index1, size_t node_index2, EdgeT const & camera_edge)
{
    std::tr1::unordered_map<size_t, LemonNode>::const_iterator node_itr1 = index_node_map_.find(node_index1);
    std::tr1::unordered_map<size_t, LemonNode>::const_iterator node_itr2 = index_node_map_.find(node_index2);
    assert(node_itr1 != index_node_map_.end() && "[PointMatchGraph::addEdge] Add edge with no corresponding nodes.");
    assert(node_itr2 != index_node_map_.end() && "[PointMatchGraph::addEdge] Add edge with no corresponding nodes.");
    LemonNode const & node1 = node_itr1->second;
    LemonNode const & node2 = node_itr2->second;
    LemonEdge const & edge = lemon_graph_.addEdge(node1, node2);
    edge_map_[edge] = camera_edge;

    std::tr1::unordered_map<size_t, std::tr1::unordered_map<size_t, LemonEdge> >::iterator edge_itr1 = index_edge_map_.find(node_index1);
    std::tr1::unordered_map<size_t, std::tr1::unordered_map<size_t, LemonEdge> >::iterator edge_itr2 = index_edge_map_.find(node_index2);

    if(edge_itr1 != index_edge_map_.end())
    {
        std::tr1::unordered_map<size_t, LemonEdge> & edge_map1 = edge_itr1->second;
        std::tr1::unordered_map<size_t, LemonEdge>::iterator edge_itr12 = edge_map1.find(node_index2);
        assert(edge_itr12 == edge_map1.end() && "[CameraDefaulGraph::addEdge] Add existed edge.");
        edge_map1.insert(std::make_pair(node_index2, edge));
    }
    else {
        std::tr1::unordered_map<size_t, LemonEdge> edge_map12;
        edge_map12.insert(std::make_pair(node_index2, edge));
        index_edge_map_.insert(std::make_pair(node_index1, edge_map12));
    }

    if(edge_itr2 != index_edge_map_.end())
    {
        std::tr1::unordered_map<size_t, LemonEdge> & edge_map2 = edge_itr2->second;
        std::tr1::unordered_map<size_t, LemonEdge>::iterator edge_itr21 = edge_map2.find(node_index1);
        assert(edge_itr21 == edge_map2.end() && "[CameraDefaulGraph::addEdge] Add existed edge.");
        edge_map2.insert(std::make_pair(node_index1, edge));
    }
    else {
        std::tr1::unordered_map<size_t, LemonEdge> edge_map21;
        edge_map21.insert(std::make_pair(node_index1, edge));
        index_edge_map_.insert(std::make_pair(node_index2, edge_map21));
    }
}

bool PointMatchGraph::edgeExist(size_t node_index1, size_t node_index2) const
{
    bool edge_exist1 = false, edge_exist2 = false;
    std::tr1::unordered_map<size_t, std::tr1::unordered_map<size_t, LemonEdge> >::const_iterator edge_itr1 = index_edge_map_.find(node_index1);
    std::tr1::unordered_map<size_t, std::tr1::unordered_map<size_t, LemonEdge> >::const_iterator edge_itr2 = index_edge_map_.find(node_index2);
    if(edge_itr1 != index_edge_map_.end())
    {
        std::tr1::unordered_map<size_t, LemonEdge> const & edge_map1 = edge_itr1->second;
        std::tr1::unordered_map<size_t, LemonEdge>::const_iterator edge_itr12 = edge_map1.find(node_index2);
        edge_exist1 = (edge_itr12 != edge_map1.end());
    }

    if(edge_itr2 != index_edge_map_.end())
    {
        std::tr1::unordered_map<size_t, LemonEdge> const & edge_map2 = edge_itr2->second;
        std::tr1::unordered_map<size_t, LemonEdge>::const_iterator edge_itr21 = edge_map2.find(node_index1);
        edge_exist2 = (edge_itr21 != edge_map2.end());
    }

    assert(edge_exist1 == edge_exist2 && "[PointMatchGraph::edgeExist] Index edge map is inconsistent");
    return edge_exist1 && edge_exist2;
}

PointMatchGraph::EdgeT & PointMatchGraph::edge(size_t node_index1, size_t node_index2)
{
    std::tr1::unordered_map<size_t, std::tr1::unordered_map<size_t, LemonEdge> >::iterator edge_itr1 = index_edge_map_.find(node_index1);
    std::tr1::unordered_map<size_t, std::tr1::unordered_map<size_t, LemonEdge> >::iterator edge_itr2 = index_edge_map_.find(node_index2);

    assert(edge_itr1 != index_edge_map_.end() && "[PointMatchGraph::edge] Edge does not exist in index edge map.");
    std::tr1::unordered_map<size_t, LemonEdge> & edge_map1 = edge_itr1->second;
    std::tr1::unordered_map<size_t, LemonEdge>::iterator edge_itr12 = edge_map1.find(node_index2);
    assert(edge_itr12 != edge_map1.end() && "[PointMatchGraph::edge] Edge does not exist in index edge map.");

    assert(edge_itr2 != index_edge_map_.end() && "[PointMatchGraph::edge] Edge does not exist in index edge map.");
    std::tr1::unordered_map<size_t, LemonEdge> & edge_map2 = edge_itr2->second;
    std::tr1::unordered_map<size_t, LemonEdge>::iterator edge_itr21 = edge_map2.find(node_index1);
    assert(edge_itr21 != edge_map2.end() && "[PointMatchGraph::edge] Edge does not exist in index edge map.");

    LemonEdge const & edge = edge_itr12->second;
    return edge_map_[edge];
}

PointMatchGraph::EdgeT const & PointMatchGraph::edge(size_t node_index1, size_t node_index2) const
{
    std::tr1::unordered_map<size_t, std::tr1::unordered_map<size_t, LemonEdge> >::const_iterator edge_itr1 = index_edge_map_.find(node_index1);
    std::tr1::unordered_map<size_t, std::tr1::unordered_map<size_t, LemonEdge> >::const_iterator edge_itr2 = index_edge_map_.find(node_index2);

    assert(edge_itr1 != index_edge_map_.end() && "[CameraDefaultGraph::edge] Edge does not exist in index edge map.");
    std::tr1::unordered_map<size_t, LemonEdge> const & edge_map1 = edge_itr1->second;
    std::tr1::unordered_map<size_t, LemonEdge>::const_iterator edge_itr12 = edge_map1.find(node_index2);
    assert(edge_itr12 != edge_map1.end() && "[CameraDefaultGraph::edge] Edge does not exist in index edge map.");

    assert(edge_itr2 != index_edge_map_.end() && "[CameraDefaultGraph::edge] Edge does not exist in index edge map.");
    std::tr1::unordered_map<size_t, LemonEdge> const & edge_map2 = edge_itr2->second;
    std::tr1::unordered_map<size_t, LemonEdge>::const_iterator edge_itr21 = edge_map2.find(node_index1);
    assert(edge_itr21 != edge_map2.end() && "[CameraDefaultGraph::edge] Edge does not exist in index edge map.");

    LemonEdge const & edge = edge_itr12->second;
    return edge_map_[edge];
}

void PointMatchGraph::nodePairIDs(std::vector<std::pair<size_t, size_t> > & pair_indexes) const
{
    for(std::tr1::unordered_map<size_t, std::tr1::unordered_map<size_t, LemonEdge> >::const_iterator edge_itr1 = index_edge_map_.begin()
        ; edge_itr1 != index_edge_map_.end(); edge_itr1++)
    {
        size_t const node_index1 = edge_itr1->first;
        std::tr1::unordered_map<size_t, LemonEdge> const & edge_map1 = edge_itr1->second;
        for(std::tr1::unordered_map<size_t, LemonEdge>::const_iterator edge_itr12 = edge_map1.begin()
            ; edge_itr12 != edge_map1.end(); edge_itr12++)
        {
            size_t const node_index2 = edge_itr12->first;
            if(node_index1 < node_index2)
            {
                pair_indexes.push_back(std::make_pair(node_index1, node_index2));
            }
        }
    }

    std::sort(pair_indexes.begin(), pair_indexes.end());
}
