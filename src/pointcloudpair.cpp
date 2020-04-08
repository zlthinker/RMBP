#include "pointcloudpair.h"
#include "kdtree.h"

#include <set>
#include <fstream>

PointCloudPair::PointCloudPair(std::tr1::unordered_map<size_t, V3d> const & coords1,
                               std::tr1::unordered_map<size_t, V3d> const & coords2,
                               std::vector<std::pair<size_t, size_t> > const & match_pairs) :
    coords1_(coords1), coords2_(coords2), match_pairs_(match_pairs)
{
    init_inlier_prob_.resize(match_pairs.size(), 0.5);
    LoadMatches(match_pairs, init_inlier_prob_);
}

PointCloudPair::PointCloudPair(std::tr1::unordered_map<size_t, V3d> const & coords1,
                               std::tr1::unordered_map<size_t, V3d> const & coords2,
                               std::vector<std::pair<size_t, size_t> > const & match_pairs,
                               std::vector<double> const & init_inlier_prob) :
    coords1_(coords1), coords2_(coords2), match_pairs_(match_pairs), init_inlier_prob_(init_inlier_prob)
{
    LoadMatches(match_pairs, init_inlier_prob);
}

PointCloudPair::~PointCloudPair() {}

bool PointCloudPair::LoadMatches(std::vector<std::pair<size_t, size_t> > const & match_pairs,
                                 std::vector<double> const & init_inlier_probs)
{
    assert(match_pairs.size() == init_inlier_probs.size() && "PointCloudPair::LoadMatches");
    for (size_t midx = 0; midx < match_pairs.size(); midx++)
    {
        size_t id1 = match_pairs[midx].first;
        size_t id2 = match_pairs[midx].second;
        double init_prob = init_inlier_probs[midx];

        if (coords1_.find(id1) == coords1_.end()) {
            ERROR("[PointCloudPair::LoadMatches] Point not exists in the first point cloud: " << id1 << "\n");
            return false;
        }
        if (coords2_.find(id2) == coords2_.end()) {
            ERROR("[PointCloudPair::LoadMatches] Point not exists in the second point cloud: " << id2 << "\n");
            return false;
        }
        // skip duplicated correspondences
        if (point2point1_.find(id1) != point2point1_.end()) { continue; }
        if (point2point2_.find(id2) != point2point2_.end()) { continue; }
        point2point1_[id1] = id2;
        point2point2_[id2] = id1;
        match_graph_.addNode(id1, PointMatchNode(id1, id2, init_prob));
    }

    return true;
}

V3d PointCloudPair::GetCoord(bool is_first, size_t point_index) const
{
    std::tr1::unordered_map<size_t, V3d> const & coord = is_first ? coords1_ : coords2_;
    return coord.find(point_index)->second;
}

/*!
 * @brief Two points are neighbors if they are k-NN neighbors
 * @param neighb_points: The neighboring points are sorted in increasing distance
 */
void PointCloudPair::GetSpatialNeighbPoints(bool is_first,
                                            const size_t num_neighbs,
                                            std::tr1::unordered_map<size_t, std::vector<size_t> > & neighb_points)
{
    // build kd-tree
    KDTree kd_tree;

    int dimension = 3;
    double ** coord_mat = new double*[match_pairs_.size()];
    for(int i = 0; i < match_pairs_.size(); i++)
    {
        size_t point_index = is_first ? match_pairs_[i].first : match_pairs_[i].second;
        V3d coord = GetCoord(is_first, point_index);
        coord_mat[i] = new double[dimension];
        for(int d = 0; d < dimension; d++) { coord_mat[i][d] = coord[d]; }
    }
    kd_tree.BuildIndex(coord_mat, match_pairs_.size(), dimension);

    neighb_points.clear();

    for(int i = 0; i < match_pairs_.size(); i++)
    {
        size_t point_index = is_first ? match_pairs_[i].first : match_pairs_[i].second;
        V3d coord = GetCoord(is_first, point_index);
        std::vector<double> query;
        for (int j = 0; j < 3; j++) { query.push_back(coord[j]); }
        std::vector<int> neighb_idxs;
        std::vector<double> neighb_dists;
        neighb_idxs.resize(num_neighbs);
        neighb_dists.resize(num_neighbs);
        kd_tree.KnnSearch(query, neighb_idxs, neighb_dists, num_neighbs);

        std::vector<int> radius_neighb_idxs;
        std::vector<double> radius_neighb_dists;
        kd_tree.RadiusSearch(query, radius_neighb_idxs, radius_neighb_dists, bp_param_.pos_neighb_distance);
        neighb_idxs.insert(neighb_idxs.end(), radius_neighb_idxs.begin(), radius_neighb_idxs.end());

        std::vector<size_t> valid_neighbs_idxs;
        valid_neighbs_idxs.reserve(neighb_idxs.size());
        // filter out the query point
        for (int j = 0; j < neighb_idxs.size(); j++)
        {
            int idx = neighb_idxs[j];
            size_t neighb_point_index = is_first ? match_pairs_[idx].first : match_pairs_[idx].second;
            if (point_index != neighb_point_index)
            {
                valid_neighbs_idxs.push_back(neighb_point_index);
            }
        }
        neighb_points[point_index] = valid_neighbs_idxs;
    }

    for(int i = 0; i < match_pairs_.size(); i++)
    {
        delete [] coord_mat[i];
    }
    delete [] coord_mat;
}

/*!
 * @brief Two matches are neighbors if the two points in either point clouds are k-NN neighbors
 */
void PointCloudPair::GetSpatialNeighbMatches(std::tr1::unordered_map<size_t, std::vector<size_t> > & positive_match_map,
                                             std::tr1::unordered_map<size_t, std::vector<size_t> > & negative_match_map)
{
    std::cout << "GetSpatialNeighbMatches...\n";
    size_t const num_neighbs_pos = std::min(bp_param_.pos_neighb_bound, size_t(match_pairs_.size() * bp_param_.pos_neighb_percent));
    size_t const num_neighbs_neg = std::max(bp_param_.neg_neighb_bound, size_t(match_pairs_.size() * bp_param_.neg_neighb_percent));
    std::tr1::unordered_map<size_t, std::vector<size_t> > neighb_points1, neighb_points2;
    {
        GetSpatialNeighbPoints(true, num_neighbs_pos, neighb_points1);
        GetSpatialNeighbPoints(false, num_neighbs_pos, neighb_points2);
    }

    positive_match_map.clear();
    negative_match_map.clear();
    for (int i = 0; i < match_pairs_.size(); i++)
    {
        size_t point_index1 = match_pairs_[i].first;
        size_t point_index2 = match_pairs_[i].second;
        std::vector<size_t> const & id1_neighbs = neighb_points1[point_index1];
        std::vector<size_t> const & id2_neighbs = neighb_points2[point_index2];
        std::vector<size_t> positive_match_pairs, negative_match_pairs;

        std::set<size_t> id1_neighb_pair_set;
        for (size_t i = 0; i < id1_neighbs.size(); i++)
        {
            size_t id1_neighb = id1_neighbs[i];
            size_t id1_neighb_pair = point2point1_[id1_neighb];
            id1_neighb_pair_set.insert(id1_neighb_pair);
        }
        std::set<size_t> id2_neighb_pair_set;
        for (size_t i = 0; i < id2_neighbs.size(); i++)
        {
            size_t id2_neighb = id2_neighbs[i];
            size_t id2_neighb_pair = point2point2_[id2_neighb];
            id2_neighb_pair_set.insert(id2_neighb_pair);
        }

        std::vector<size_t> mutual_id2_neighbs;
        std::set<size_t> id2_neighb_set(id2_neighbs.begin(),
                                        std::min(id2_neighbs.begin() + num_neighbs_pos, id2_neighbs.end()));
        std::set_intersection(id1_neighb_pair_set.begin(), id1_neighb_pair_set.end(),
                              id2_neighb_set.begin(), id2_neighb_set.end(),
                              std::back_inserter(mutual_id2_neighbs));
        for (int i = 0; i < mutual_id2_neighbs.size(); i++)
        {
            size_t id2_neighb = mutual_id2_neighbs[i];
            size_t id1_neighb = point2point2_[id2_neighb];
            positive_match_pairs.push_back(id1_neighb);
        }

        std::set<size_t>::iterator it1 = id1_neighb_pair_set.begin();
        for (; it1 != id1_neighb_pair_set.end(); it1++)
        {
            size_t id1_neighb_pair = *it1;
            V3d id1_neighb_pair_coord = GetCoord(false, id1_neighb_pair);
            V3d id2_coord = GetCoord(false, point_index2);
            double distance = L2Distance(id1_neighb_pair_coord, id2_coord);
            if (distance > bp_param_.neg_neighb_distance)
            {
                size_t id1_neighb = point2point2_[id1_neighb_pair];
                negative_match_pairs.push_back(id1_neighb);
            }
        }

        std::set<size_t>::iterator it2 = id2_neighb_pair_set.begin();
        for (; it2 != id2_neighb_pair_set.end(); it2++)
        {
            size_t id2_neighb_pair = *it2;
            V3d id2_neighb_pair_coord = GetCoord(true, id2_neighb_pair);
            V3d id1_coord = GetCoord(true, point_index1);
            double distance = L2Distance(id2_neighb_pair_coord, id1_coord);
            if (distance > bp_param_.neg_neighb_distance)
            {
                negative_match_pairs.push_back(id2_neighb_pair);
            }
        }

//        if (positive_match_pairs.size() > bp_param_.max_pos_neighb_num)
//        {
//            positive_match_pairs.resize(bp_param_.max_pos_neighb_num);
//        }
//        if (negative_match_pairs.size() > bp_param_.max_neg_neighb_num)
//        {
//            negative_match_pairs.resize(bp_param_.max_neg_neighb_num);
//        }

        positive_match_map[point_index1] = positive_match_pairs;
        negative_match_map[point_index1] = negative_match_pairs;
    }
}

void PointCloudPair::BuildPointMatchGraphEdges()
{
    std::tr1::unordered_map<size_t, std::vector<size_t> >  positive_match_map, negative_match_map;
    GetSpatialNeighbMatches(positive_match_map, negative_match_map);

    std::tr1::unordered_map<size_t, std::vector<size_t> >::iterator it = positive_match_map.begin();
    for (; it != positive_match_map.end(); it++)
    {
        size_t point_index1 = it->first;
        std::vector<size_t> const& id1_neighbs = it->second;
        for (int j = 0; j < id1_neighbs.size(); j++)
        {
            size_t id1_neighb = id1_neighbs[j];
            if (!match_graph_.edgeExist(point_index1, id1_neighb))
            {
                PointMatchEdge edge(point_index1, id1_neighb, true);
                match_graph_.addEdge(point_index1, id1_neighb, edge);
            }
        }
    }
    size_t num_pos_edges = match_graph_.edgeNum();

    it = negative_match_map.begin();
    for (; it != negative_match_map.end(); it++)
    {
        size_t point_index1 = it->first;
        std::vector<size_t> const& id1_neighbs = it->second;
        for (int j = 0; j < id1_neighbs.size(); j++)
        {
            size_t id1_neighb = id1_neighbs[j];
            if (!match_graph_.edgeExist(point_index1, id1_neighb))
            {
                PointMatchEdge edge(point_index1, id1_neighb, false);
                match_graph_.addEdge(point_index1, id1_neighb, edge);
            }
        }
    }
    size_t num_neg_edges = match_graph_.edgeNum() - num_pos_edges;
    std::cout << "Match graph is built: # nodes = " << match_graph_.nodeNum()
              << ", # positive edges = " << num_pos_edges << "\n"
              << ", # negative edges = " << num_neg_edges << "\n";
}

void PointCloudPair::InitMessage(std::tr1::unordered_map<size_t, V2d> & measurement_message_map,
                                 std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> & message_map)
{
    std::vector<size_t> node_indexes;
    match_graph_.nodeIDs(node_indexes);
    for (size_t idx = 0; idx < node_indexes.size(); idx++)
    {
        size_t node_index = node_indexes[idx];
        PointMatchNode const & node = match_graph_.node(node_index);
        V2d measurement_message = {1.0 - node.prob(), node.prob()};
        measurement_message_map[node_index] = measurement_message;

        std::vector<size_t> neighb_node_indexes;
        match_graph_.adjacentNodeIDs(node_index, neighb_node_indexes);
        for (size_t nidx = 0; nidx < neighb_node_indexes.size(); nidx++)
        {
            size_t neighb_index = neighb_node_indexes[nidx];
            V2d message = {0.5, 0.5};
            message_map[std::make_pair(node_index, neighb_index)] = message;
        }
    }
}

void PointCloudPair::InitBelief(std::tr1::unordered_map<size_t, V2d> & belief)
{
    std::vector<size_t> node_indexes;
    match_graph_.nodeIDs(node_indexes);
    for (size_t idx = 0; idx < node_indexes.size(); idx++)
    {
        size_t node_index = node_indexes[idx];
        V2d init_belief = {0.5, 0.5};
        belief[node_index] = init_belief;
    }
}

/*!
 * @brief Compute hyperparam lambda to gurantee convergence based on Simon's condition
 */
void PointCloudPair::InitCompatibilityMatrix()
{
    size_t max_neighb_num = 0;

    std::vector<size_t> node_indexes;
    match_graph_.nodeIDs(node_indexes);
    for (size_t idx = 0; idx < node_indexes.size(); idx++)
    {
        size_t node_index = node_indexes[idx];
        std::vector<size_t> neighb_node_indexes;
        match_graph_.adjacentNodeIDs(node_index, neighb_node_indexes);
        if (neighb_node_indexes.size() > max_neighb_num)
        {
            max_neighb_num = neighb_node_indexes.size();
        }
    }

    bp_param_.lambda = exp(2.0 / max_neighb_num);
}

bool PointCloudPair::UpdateMessage(size_t index1, size_t index2,
                                   std::tr1::unordered_map<size_t, V2d> const & measurement_message_map,
                                   std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> const & last_message_map,
                                   std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> & update_message_map)
{
    std::tr1::unordered_map<size_t, V2d>::const_iterator it1 = measurement_message_map.find(index1);
    assert(it1 != measurement_message_map.end());
    V2d message = it1->second;

    std::vector<size_t> neighb_node_indexes;
    match_graph_.adjacentNodeIDs(index1, neighb_node_indexes);

    for (size_t nidx = 0; nidx < neighb_node_indexes.size(); nidx++)
    {
        size_t neighb_index = neighb_node_indexes[nidx];
        if (neighb_index == index2) { continue; }
        std::tr1::unordered_map< std::pair<size_t, size_t>, V2d>::const_iterator it
                = last_message_map.find(std::make_pair(neighb_index, index1));
        assert(it != last_message_map.end());
        V2d neighb_message = it->second;
        message[0] *= neighb_message[0];
        message[1] *= neighb_message[1];
    }

    double compatibility_matrix[2][2];
    PointMatchEdge const & edge = match_graph_.edge(index1, index2);
    if (edge.isMutual())
    {
        compatibility_matrix[0][0] = compatibility_matrix[0][1] = compatibility_matrix[1][0] = 1.0;
        compatibility_matrix[1][1] = bp_param_.lambda;
    }
    else
    {
        compatibility_matrix[0][0] = compatibility_matrix[0][1] = compatibility_matrix[1][0] = bp_param_.lambda;
        compatibility_matrix[1][1] = 1.0;
    }

    std::tr1::unordered_map< std::pair<size_t, size_t>, V2d>::const_iterator it2
            = last_message_map.find(std::make_pair(index1, index2));
    assert(it2 != last_message_map.end());
    V2d update_message;
    update_message[0] = compatibility_matrix[0][0] * message[0] + compatibility_matrix[0][1] * message[1];
    update_message[1] = compatibility_matrix[1][0] * message[0] + compatibility_matrix[1][1] * message[1];
    double sum = update_message[0] + update_message[1];
    update_message[0] = update_message[0] / sum;
    update_message[1] = update_message[1] / sum;
    update_message_map[std::make_pair(index1, index2)] = update_message;
    return true;
}

bool PointCloudPair::UpdateMessage(std::tr1::unordered_map<size_t, V2d> const & measurement_message_map,
                                   std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> const & last_message_map,
                                   std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> & update_message_map)
{
    std::vector<size_t> node_indexes;
    match_graph_.nodeIDs(node_indexes);

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (size_t idx = 0; idx < node_indexes.size(); idx++)
    {
        size_t node_index = node_indexes[idx];

        std::vector<size_t> neighb_node_indexes;
        match_graph_.adjacentNodeIDs(node_index, neighb_node_indexes);
        for (size_t nidx = 0; nidx < neighb_node_indexes.size(); nidx++)
        {
            size_t neighb_index = neighb_node_indexes[nidx];
            UpdateMessage(neighb_index, node_index, measurement_message_map, last_message_map, update_message_map);
        }
    }

    return true;
}

void PointCloudPair::ComputeBelief(std::tr1::unordered_map<size_t, V2d> const & measurement_message_map,
                                   std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> const & message_map,
                                   std::tr1::unordered_map<size_t, V2d> & belief)
{
    belief.clear();

    std::vector<size_t> node_indexes;
    match_graph_.nodeIDs(node_indexes);

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (size_t idx = 0; idx < node_indexes.size(); idx++)
    {
        size_t node_index = node_indexes[idx];
        std::tr1::unordered_map<size_t, V2d>::const_iterator it1 = measurement_message_map.find(node_index);
        assert(it1 != measurement_message_map.end());
        V2d message = it1->second;

        std::vector<size_t> neighb_node_indexes;
        match_graph_.adjacentNodeIDs(node_index, neighb_node_indexes);
        for (size_t nidx = 0; nidx < neighb_node_indexes.size(); nidx++)
        {
            size_t neighb_index = neighb_node_indexes[nidx];
            std::tr1::unordered_map< std::pair<size_t, size_t>, V2d>::const_iterator it
                    = message_map.find(std::make_pair(neighb_index, node_index));
            assert(it != message_map.end());
            V2d neighb_message = it->second;
            message[0] *= neighb_message[0];
            message[1] *= neighb_message[1];
        }
        double sum = message[0] + message[1];
        message[0] = message[0] / sum;
        message[1] = message[1] / sum;
        belief[node_index] = message;
    }
}

bool PointCloudPair::StopCondition(std::tr1::unordered_map<size_t, V2d> const & belief1,
                                   std::tr1::unordered_map<size_t, V2d> const & belief2)
{
    std::tr1::unordered_map<size_t, V2d>::const_iterator it1 = belief1.begin();
    std::tr1::unordered_map<size_t, V2d>::const_iterator it2 = belief2.begin();
    for (; it1 != belief1.end(); it1++, it2++)
    {
        assert(it1->first == it2->first);
        V2d prob1 = it1->second;
        V2d prob2 = it2->second;
        if (std::abs(prob1[0] - prob2[1]) > bp_param_.delta_belief)
        {
            return false;
        }
    }

    return true;
}

void PointCloudPair::RefineMatchPairs(std::tr1::unordered_map<size_t, V2d> const & belief,
                                      std::vector<std::pair<size_t, size_t> > & refine_match_pairs,
                                      std::vector<double> & refine_belief)
{
    std::vector<size_t> node_indexes;
    match_graph_.nodeIDs(node_indexes);

    for (size_t idx = 0; idx < node_indexes.size(); idx++)
    {
        size_t node_index = node_indexes[idx];
        PointMatchNode const & node = match_graph_.node(node_index);

        std::tr1::unordered_map<size_t, V2d>::const_iterator it = belief.find(node_index);
        assert(it != belief.end());
        V2d result = it->second;
        std::cout << node_index << ", belief: " << result[1] << "\n";
        if (result[1] >= bp_param_.belief_threshold)
        {
            refine_match_pairs.push_back(std::make_pair(node.index(), node.pairIndex()));
            refine_belief.push_back(result[1]);
        }
    }
}

bool PointCloudPair::BeliefPropagation(std::vector<std::pair<size_t, size_t> > & match_pairs,
                                       std::vector<double> & refine_belief)
{
    if (match_pairs_.size() <= bp_param_.neg_neighb_bound)
    {
        WARNING("The number of match pairs is deficient: " << match_pairs_.size() << "\n");
        return false;
    }

    BuildPointMatchGraphEdges();

    std::tr1::unordered_map<size_t, V2d> measurement_message_map;
    std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> message_map;
    std::tr1::unordered_map<size_t, V2d> belief;

    InitMessage(measurement_message_map, message_map);
    InitBelief(belief);
    InitCompatibilityMatrix();

    for (size_t idx = 0; idx < bp_param_.max_iteration; idx++)
    {
        std::cout << "*********************** iteration " << idx << " ***********************\n";
        std::tr1::unordered_map< std::pair<size_t, size_t>, V2d> last_message_map = message_map;

        if (!UpdateMessage(measurement_message_map, last_message_map, message_map))
        {
            return false;
        }

        std::tr1::unordered_map<size_t, V2d> last_belief = belief;
        ComputeBelief(measurement_message_map, message_map, belief);
        if (StopCondition(last_belief, belief))
        {
            break;
        }
    }

    match_pairs.clear();
    RefineMatchPairs(belief, match_pairs, refine_belief);

    return true;
}

bool RMBP(std::tr1::unordered_map<size_t, V3d> const & coords1,
          std::tr1::unordered_map<size_t, V3d> const & coords2,
          std::vector<std::pair<size_t, size_t> > const & match_pairs,
          std::vector<double> const & init_inlier_probs,
          std::vector<std::pair<size_t, size_t> > & refine_match_pairs,
          std::vector<double> & refine_belief,
          double belief_threshold,
          size_t max_iteration)
{
    PointCloudPair pc_pair(coords1, coords2, match_pairs, init_inlier_probs);
    pc_pair.SetBeliefThreshold(belief_threshold);
    pc_pair.SetMaxIteration(max_iteration);

    if (!pc_pair.BeliefPropagation(refine_match_pairs, refine_belief))
    {
        return false;
    }

    return true;
}

bool RMBP(std::tr1::unordered_map<size_t, V2d> const & coords1,
          std::tr1::unordered_map<size_t, V2d> const & coords2,
          std::vector<std::pair<size_t, size_t> > const & match_pairs,
          std::vector<double> const & init_inlier_probs,
          std::vector<std::pair<size_t, size_t> > & refine_match_pairs,
          std::vector<double> & refine_belief,
          double belief_threshold,
          size_t max_iteration)
{
    std::tr1::unordered_map<size_t, V3d> coords1_3d, coords2_3d;
    std::tr1::unordered_map<size_t, V2d>::const_iterator it = coords1.begin();
    for (; it != coords1.end(); it++)
    {
        size_t index = it->first;
        V2d coord = it->second;
        V3d coord_3d = {coord[0], coord[1], 0.0};
        coords1_3d[index] = coord_3d;
    }
    it = coords2.begin();
    for (; it != coords2.end(); it++)
    {
        size_t index = it->first;
        V2d coord = it->second;
        V3d coord_3d = {coord[0], coord[1], 0.0};
        coords2_3d[index] = coord_3d;
    }

    PointCloudPair pc_pair(coords1_3d, coords2_3d, match_pairs, init_inlier_probs);
    pc_pair.SetBeliefThreshold(belief_threshold);
    pc_pair.SetMaxIteration(max_iteration);

    if (!pc_pair.BeliefPropagation(refine_match_pairs, refine_belief))
    {
        return false;
    }

    return true;
}

