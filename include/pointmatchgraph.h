#ifndef POINTMATCHGRPAH_H
#define POINTMATCHGRPAH_H

#include <tr1/unordered_map>
#include <cmath>

#include "lemon/list_graph.h"
#include "lemon/maps.h"

class PointMatchNode
{
public:
    PointMatchNode() : index_(0), pair_index_(0) {}

    PointMatchNode(size_t index1, size_t index2)
        : index_(index1), pair_index_(index2), prob_(1.0) {}

    PointMatchNode(size_t index1, size_t index2, double prob)
        : index_(index1), pair_index_(index2), prob_(prob) {
    }

    inline size_t index() const { return index_; }
    inline size_t pairIndex() const { return pair_index_; }
    inline double prob() const { return prob_; }
    inline void setProb(double prob) { prob_ = prob; }

private:
    size_t index_;
    size_t pair_index_;
    double prob_;   // Probability of being an inlier for unary nodes
};

class PointMatchEdge
{
public:
    PointMatchEdge() {}

    PointMatchEdge(size_t index1, size_t index2, bool is_mutual)
        : index1_(index1), index2_(index2), is_mutual_(is_mutual) {}

    PointMatchEdge(const PointMatchEdge& edge)
        : index1_(edge.index1()), index2_(edge.index2()), is_mutual_(edge.isMutual()) {}

    inline size_t index1() const { return index1_; }
    inline size_t index2() const { return index2_; }
    inline bool isMutual() const { return is_mutual_; }

private:
    size_t index1_;
    size_t index2_;
    bool is_mutual_;
};

class PointMatchGraph
{
public:
    typedef typename lemon::ListGraph LemonGraph;
    typedef typename LemonGraph::Node LemonNode;
    typedef typename LemonGraph::Edge LemonEdge;
    typedef PointMatchNode NodeT;
    typedef PointMatchEdge EdgeT;

public:
    PointMatchGraph() : node_map_(lemon_graph_), edge_map_(lemon_graph_) {}
    inline size_t nodeNum() const { return index_node_map_.size(); }
    void addNode(size_t node_index, NodeT const & camera_node);
    bool nodeExist(size_t node_index) const;
    NodeT & node(size_t node_index);
    NodeT const & node(size_t node_index) const;
    void nodeIDs(std::vector<size_t> & node_indexes) const;
    int nodeDegree(size_t node_index) const;
    void adjacentNodeIDs(size_t node_index, std::vector<size_t> & node_indexes) const;

    size_t edgeNum() const;
    void addEdge(size_t node_index1, size_t node_index2, EdgeT const & camera_edge);
    bool edgeExist(size_t node_index1, size_t node_index2) const;
    EdgeT & edge(size_t node_index1, size_t node_index2);
    EdgeT const & edge(size_t node_index1, size_t node_index2) const;
    void nodePairIDs(std::vector<std::pair<size_t, size_t> > & pair_indexes) const;

private:
    LemonGraph lemon_graph_;
    std::tr1::unordered_map<size_t, LemonNode> index_node_map_;
    std::tr1::unordered_map<size_t, std::tr1::unordered_map<size_t, LemonEdge> > index_edge_map_;
    LemonGraph::NodeMap<PointMatchNode> node_map_;
    LemonGraph::EdgeMap<PointMatchEdge> edge_map_;

};

#endif // POINTMATCHGRPAH_H
