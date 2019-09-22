import cv2 as cv
import matplotlib.pyplot as plt
import argparse

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
        print val1, val2, val3, val4
        kp1.append(cv.KeyPoint(x=val1, y=val2, _size=1))
        kp2.append(cv.KeyPoint(x=val3, y=val4, _size=1))
        matches.append(cv.DMatch(_queryIdx=i-1, _trainIdx=i-1, _distance=0.0))
    return kp1, kp2, matches

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

