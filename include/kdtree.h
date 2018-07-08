#ifndef KDTREE_H
#define KDTREE_H

#include <tr1/unordered_map>
#include <vector>
#include "ann/include/ANN/ANN.h"

class ANNkd_tree;

class KDTree
{
public:
    KDTree();
    virtual ~KDTree();

    int Dim() const;
    int PointNum() const;
    bool BuildIndex(double ** pa, int n, int d);
    bool KnnSearch(std::vector<double> & query,
                   std::vector<int> & result_indices,
                   std::vector<double> & result_dists,
                   const int num_neighbs);

private:
    ANNkd_tree * tree_;
    std::tr1::unordered_map<int, std::vector<int> > leaf_point_indexes_;
    std::tr1::unordered_map<int, std::vector<double> > bound_boxes_;

};

#endif
