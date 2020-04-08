#include "pointcloudpair.h"
#include <fstream>
#include <string.h>

struct RMBPParam
{
    RMBPParam() :
        match_file(""),
        belief_threshold(0.45),
        log_file(""),
        max_iteration(20) {}

    std::string match_file;
    double belief_threshold;
    std::string log_file;
    size_t max_iteration;
};

void PrintHelp()
{
    std::cout << "<exe> <match_file> <output_file>\n"
              << "--iteration <val> : Set maximum iteration, default val = 20 \n"
              << "--belief <val> : Set belief threshold that distinguishes inlier/outlier, default val = 0.45 \n";
}

void ParseCommand(int argc, char* argv[], RMBPParam & param)
{
    if (argc < 3)
    {
        PrintHelp();
        exit(-1);
    }

    param.match_file = argv[1];
    param.log_file = argv[2];

    size_t i = 3;
    while (i < argc)
    {
        std::cout << i << ", option = " << argv[i] << "\n";
        if (strcmp(argv[i], "--belief") == 0)
        {
            i++;
            param.belief_threshold = atof(argv[i++]);
            std::cout << "[ParseCommand] Belief threshold is set to " << param.belief_threshold << "\n";
        }
        else if (strcmp(argv[i], "--iter") == 0)
        {
            param.max_iteration = atoi(argv[i++]);
            std::cout << "[ParseCommand] Max iteration is set to " << param.max_iteration << "\n";
        }
        else
        {
            std::cout << "Invalid argument: " << argv[i] << "\n";
            PrintHelp();
            exit(-1);
        }
    }
}


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
    RMBPParam param;
    ParseCommand(argc, argv, param);

    std::tr1::unordered_map<size_t, V3d> coords1;
    std::tr1::unordered_map<size_t, V3d> coords2;
    std::vector<std::pair<size_t, size_t> > match_pairs;
    std::vector<double> init_inlier_probs;
    ReadMatchFile(param.match_file, coords1, coords2, match_pairs, init_inlier_probs);

    std::vector<std::pair<size_t, size_t> > refine_match_pairs;
    std::vector<double> refine_belief;
    RMBP(coords1, coords2, match_pairs, init_inlier_probs, refine_match_pairs, refine_belief, param.belief_threshold, param.max_iteration);

    std::cout << "# refine match pairs after RMBP: " << refine_match_pairs.size() << "\n";
    
    if (param.log_file != "")
    {
        std::ofstream fout(param.log_file);
        fout << refine_match_pairs.size() << "\n";
        for (size_t midx = 0; midx < refine_match_pairs.size(); midx++)
        {
            size_t index = refine_match_pairs[midx].first;
            V3d coord1 = coords1[index];
            V3d coord2 = coords2[index];
            fout << coord1[0] << " " << coord1[1] << " " << coord1[2] << "\t"
                              << coord2[0] << " " << coord2[1] << " " << coord2[2] << "\t"
                              << refine_belief[midx] << "\n";
        }
        fout.close();
    }
    return 0;
}
