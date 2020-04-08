#ifndef POINTCLOUDPAIR_H
#define POINTCLOUDPAIR_H

#include <vector>
#include <string>
#include <cassert>
#include <tr1/cstdlib>
#include <tr1/array>
#include <tr1/unordered_map>

#include "pointmatchgraph.h"
#include "util.h"

typedef std::tr1::array<double, 2> V2d;
typedef std::tr1::array<double, 3> V3d;

namespace std
{
namespace tr1
{
template<>
struct hash< std::pair<size_t, size_t> >
{
    size_t operator() (std::pair<size_t, size_t> const & pair) const
    {
        return pair.first << 32 | pair.second;
    }
};
}
}

class PointCloudPair
{
public:
    PointCloudPair();
    PointCloudPair(std::tr1::unordered_map<size_t, V3d> const & coords1,
                   std::tr1::unordered_map<size_t, V3d> const & coords2,
                   std::vector<std::pair<size_t, size_t> > const & match_pairs);
    PointCloudPair(std::tr1::unordered_map<size_t, V3d> const & coords1,
                   std::tr1::unordered_map<size_t, V3d> const & coords2,
                   std::vector<std::pair<size_t, size_t> > const & match_pairs,
                   std::vector<double> const & init_inlier_prob);

    ~PointCloudPair();

public:
    inline size_t PointNum1() const { return coords1_.size(); }
    inline size_t PointNum2() const { return coords2_.size(); }
    inline size_t MatchNum() const { return match_pairs_.size(); }
    inline void SetBeliefThreshold(double threshold) { bp_param_.belief_threshold = threshold; }
    inline void SetMaxIteration(size_t max_iteration) { bp_param_.max_iteration = max_iteration; }
    bool BeliefPropagation(std::vector<std::pair<size_t, size_t> > & refine_match_pairs, std::vector<double> & refine_belief);

private:
    bool LoadMatches(std::vector<std::pair<size_t, size_t> > const & match_pairs,
                     std::vector<double> const & init_inlier_probs);
    V3d GetCoord(bool is_first, size_t point_index) const;
    void GetSpatialNeighbPoints(bool is_first,
                                const size_t num_neighbs,
                                std::tr1::unordered_map<size_t, std::vector<size_t> >& neighb_points);
    void GetSpatialNeighbMatches(std::tr1::unordered_map<size_t, std::vector<size_t> > & positive_match_map,
                                 std::tr1::unordered_map<size_t, std::vector<size_t> > & negative_match_map);
    void BuildPointMatchGraphEdges();
    void InitMessage(std::tr1::unordered_map<size_t, V2d> & measurement_message_map,
                     std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> & message_map);
    void InitBelief(std::tr1::unordered_map<size_t, V2d> & belief);
    void InitCompatibilityMatrix();
    bool UpdateMessage(size_t index1, size_t index2,
                       std::tr1::unordered_map<size_t, V2d> const & measurement_message_map,
                       std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> const & last_message_map,
                       std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> & update_message_map);
    bool UpdateMessage(std::tr1::unordered_map<size_t, V2d> const & measurement_message_map,
                       std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> const & last_message_map,
                       std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> & update_message_map);
    void ComputeBelief(std::tr1::unordered_map<size_t, V2d> const & measurement_message_map,
                       std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> const & message_map,
                       std::tr1::unordered_map<size_t, V2d> & belief);
    bool StopCondition(std::tr1::unordered_map<size_t, V2d> const & belief1,
                       std::tr1::unordered_map<size_t, V2d> const & belief2);
    void RefineMatchPairs(std::tr1::unordered_map<size_t, V2d> const & belief,
                          std::vector<std::pair<size_t, size_t> > & refine_match_pairs,
                          std::vector<double> & refine_belief);

private:
    struct BPParam
    {
        BPParam() :
            pos_neighb_percent(0.01),
            neg_neighb_percent(0.1),
            lambda(2.0),
            pos_neighb_bound(5),
            neg_neighb_bound(100),
            max_pos_neighb_num(10),
            max_neg_neighb_num(50),
            delta_belief(1e-3),
            max_iteration(100),
            belief_threshold(0.5) {}

        double pos_neighb_percent;
        double neg_neighb_percent;

        double lambda;

        size_t pos_neighb_bound;
        size_t neg_neighb_bound;
        size_t max_pos_neighb_num;
        size_t max_neg_neighb_num;

        double delta_belief;
        size_t max_iteration;

        double belief_threshold;
    };


private:
    std::vector<std::pair<size_t, size_t> > match_pairs_;
    std::vector<double> init_inlier_prob_;
    std::tr1::unordered_map<size_t, V3d> coords1_;
    std::tr1::unordered_map<size_t, V3d> coords2_;
    std::tr1::unordered_map<size_t, size_t> point2point1_; // <point index1, point index2>
    std::tr1::unordered_map<size_t, size_t> point2point2_; // <point index2, point index1>
    PointMatchGraph match_graph_;
    BPParam bp_param_;
};

bool RMBP(std::tr1::unordered_map<size_t, V3d> const & coords1,
          std::tr1::unordered_map<size_t, V3d> const & coords2,
          std::vector<std::pair<size_t, size_t> > const & match_pairs,
          std::vector<double> const & init_inlier_probs,
          std::vector<std::pair<size_t, size_t> > & refine_match_pairs,
          std::vector<double> & refine_belief,
          double belief_threshold = 0.5,
          size_t max_iteration = 100);

#endif // POINTCLOUDPAIR_H
