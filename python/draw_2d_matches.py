import cv2 as cv
import matplotlib.pyplot as plt
import argparse
import numpy as np

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
        val3 = float(elements[3])
        val4 = float(elements[4])
        kp1.append(cv.KeyPoint(x=val1, y=val2, _size=1))
        kp2.append(cv.KeyPoint(x=val3, y=val4, _size=1))
        matches.append(cv.DMatch(_queryIdx=i-1, _trainIdx=i-1, _distance=0.0))
    return kp1, kp2, matches

def drawMatches(img1, kp1, img2, kp2, matches):
    space = 1000
    thickness = 6
    color = (255, 0, 0)

    h1, w1, c1 = img1.shape
    h2, w2, c2 = img2.shape
    print img1.shape, img1.dtype
    assert h1 == h2
    assert c1 == c2
    concat = np.ones(shape=(h1, w1+w2+space, c1), dtype=img1.dtype) * 255
    concat[:, 0:w1, :] = img1
    concat[:, w1+space:w1+space+w2, :] = img2

    for m, n in matches:
        index1 = m.queryIdx
        index2 = m.trainIdx
        coord1 = kp1[index1].pt
        coord2 = kp2[index2].pt
        coord1 = (int(coord1[0]), int(coord1[1]))
        coord2 = (int(coord2[0])+w1+space, int(coord2[1]))
        cv.line(concat, coord1, coord2, color, thickness)
    return concat

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('image1', type=str, help="Image file1")
    parser.add_argument('image2', type=str, help="Image file2")
    parser.add_argument('match_file', type=str, help="Match file")
    args = parser.parse_args()

    img1 = cv.imread(args.image1)  # queryImage
    img2 = cv.imread(args.image2)  # trainImage
    kp1, kp2, matches = parge_match_file(args.match_file)
    img3 = cv.drawMatches(img1, kp1, img2, kp2, matches, None)
    img3 = cv.cvtColor(img3, cv.COLOR_BGR2RGB)
    plt.imshow(img3)
    plt.show()

