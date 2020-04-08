#include "kdtree.h"
#include "util.h"

#include <cassert>

KDTree::KDTree()
: tree_(NULL)
{}

KDTree::~KDTree()
{
    if (tree_ != NULL)
    {
        delete tree_;
        tree_ = NULL;
    }
}

int KDTree::Dim() const
{
    return (tree_ != NULL) ? tree_->theDim() : -1;
}

int KDTree::PointNum() const
{
    return (tree_ != NULL) ? tree_->nPoints() : -1;
}

bool KDTree::BuildIndex(double ** pa, int n, int d)
{
    if (tree_ != NULL)
    {
        delete tree_;
        tree_ = NULL;
    }
    tree_ = new ANNkd_tree(pa, n, d);

    std::vector<int> leaf_indexes;
    std::vector<std::vector<int> > leaf_point_indexes;
    std::vector<std::vector<double> > bound_boxes;

    tree_->TraverseByLevel(leaf_indexes, leaf_point_indexes, bound_boxes);

    assert(leaf_indexes.size() == leaf_point_indexes.size());
    assert(leaf_indexes.size() == bound_boxes.size());

    for(size_t idx = 0; idx < leaf_indexes.size(); idx++)
    {
        int leaf_index = leaf_indexes[idx];
        leaf_point_indexes_.insert(std::make_pair(leaf_index, leaf_point_indexes[idx]));
        bound_boxes_.insert(std::make_pair(leaf_index, bound_boxes[idx]));
    }

    return true;
}

bool KDTree::KnnSearch(std::vector<double> & query,
                       std::vector<int> & result_indices,
                       std::vector<double> & result_dists,
                       const int num_neighbs)
{
    assert(query.size() == static_cast<size_t>(Dim()));
    assert(result_indices.size() == static_cast<size_t>(num_neighbs));
    assert(result_dists.size() == static_cast<size_t>(num_neighbs));
    assert(tree_ != NULL);

    tree_->annMaxPtsVisit(0);
    tree_->annkSearch(query.begin().base(), num_neighbs, result_indices.begin().base(), result_dists.begin().base(), 0.0);
    tree_->annMaxPtsVisit(0);

    return true;
}

bool KDTree::RadiusSearch(std::vector<double> & query,
                  std::vector<int> & result_indices,
                  std::vector<double> & result_dists,
                  const double radius)
{
    assert(query.size() == static_cast<size_t>(Dim()));
    assert(tree_ != NULL);

    double square_radius = radius * radius;

    tree_->annMaxPtsVisit(0);
    int num_neighbs = tree_->annkFRSearch(query.begin().base(), square_radius, 0, NULL, NULL, 0.0);
    result_indices.clear();
    result_dists.clear();
    result_indices.resize(num_neighbs);
    result_dists.resize(num_neighbs);
    tree_->annkFRSearch(query.begin().base(), square_radius, num_neighbs, result_indices.begin().base(), result_dists.begin().base(), 0.0);
    tree_->annMaxPtsVisit(0);

    for (size_t i = 0; i < result_dists.size(); i++)
        result_dists[i] = std::sqrt(result_dists[i]);

    return true;
}
