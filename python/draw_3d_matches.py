import argparse, open3d

def read_lines(filepath):
    with open(filepath) as fin:
        lines = fin.readlines()
    lines = [line.strip() for line in lines]
    return lines

def parge_match_file(file):
    lines = read_lines(file)

    kp1=[]
    kp2=[]
    matches=[]
    for i in range(1, len(lines)):
        line = lines[i]
        elements = line.split()
        assert len(elements)==7
        val1 = float(elements[0])
        val2 = float(elements[1])
        val3 = float(elements[2])
        val4 = float(elements[3])
        val5 = float(elements[4])
        val6 = float(elements[5])
        kp1.append([val1, val2, val3])
        kp2.append([val4, val5, val6])
        matches.append([i-1, i-1])
    return kp1, kp2, matches

def draw_matches(kp1, kp2, matches):
    pcd1 = open3d.geometry.PointCloud()
    pcd1.points = open3d.utility.Vector3dVector(kp1)
    aabox = pcd1.get_axis_aligned_bounding_box()
    extent = aabox.get_extent()

    for i in range(len(kp2)):
        kp2[i][0] = kp2[i][0] + extent[0]

    points = kp1 + kp2
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(points)

    for i in range(len(matches)):
        matches[i][1] = matches[i][1] + len(kp1)
    colors = [[0, 0, 1] for i in range(len(matches))]
    line_set = open3d.geometry.LineSet(points=open3d.utility.Vector3dVector(points), lines=open3d.utility.Vector2iVector(matches))
    line_set.colors = open3d.utility.Vector3dVector(colors)

    open3d.visualization.draw_geometries([pcd, line_set])

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('match_file', type=str, help="Match file")
    args = parser.parse_args()

    kp1, kp2, matches = parge_match_file(args.match_file)
    draw_matches(kp1, kp2, matches)

