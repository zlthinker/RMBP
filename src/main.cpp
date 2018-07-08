#include "pointcloudpair.h"
#include <fstream>

bool ReadMatchFile(std::string const & match_file,
                   std::tr1::unordered_map<size_t, V3d> & coords1,
                   std::tr1::unordered_map<size_t, V3d> & coords2,
                   std::vector<std::pair<size_t, size_t> > & match_pairs,
                   std::vector<double> & init_inlier_probs)
{
    size_t match_num;
    std::ifstream fin(match_file.c_str());
    if (!fin.good())
    {
        ERROR("Match file not exists: " << match_file << "\n");
        return false;
    }

    fin >> match_num;
    for (size_t midx = 0; midx < match_num; midx++)
    {
        double x1, y1, z1, x2, y2, z2, init_prob;
        fin >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> init_prob;
        V3d point1 = {x1, y1, z1}, point2 = {x2, y2, z2};
        coords1[midx] = point1;
        coords2[midx] = point2;
        match_pairs.push_back(std::make_pair(midx, midx));
        init_inlier_probs.push_back(init_prob);
    }

    std::cout << "# putative match pairs: " << match_num << "\n";
    fin.close();
    return true;
}

int main(int argc, char* argv[])
{
    std::string match_file = argv[1];
    std::cout << "Read match file: " << match_file << "\n";

    std::tr1::unordered_map<size_t, V3d> coords1;
    std::tr1::unordered_map<size_t, V3d> coords2;
    std::vector<std::pair<size_t, size_t> > match_pairs;
    std::vector<double> init_inlier_probs;
    ReadMatchFile(match_file, coords1, coords2, match_pairs, init_inlier_probs);

    std::vector<std::pair<size_t, size_t> > refine_match_pairs;
    RMBP(coords1, coords2, match_pairs, init_inlier_probs, refine_match_pairs, 0.45, 100);

    std::cout << "# refine match pairs after RMBP: " << refine_match_pairs.size() << "\n";
    for (size_t midx = 0; midx < refine_match_pairs.size(); midx++)
    {
        std::cout << refine_match_pairs[midx].first << "\t" << refine_match_pairs[midx].second << "\n";
    }
    return 0;
}
