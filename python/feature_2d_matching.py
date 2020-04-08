import cv2 as cv
import matplotlib.pyplot as plt
import argparse
from draw_2d_matches import drawMatches

def feature_match(image_file1, image_file2, ratio_test=True, log_file=''):
    img1 = cv.imread(image_file1)          # queryImage
    img2 = cv.imread(image_file2) # trainImage

    feat = cv.xfeatures2d.SIFT_create(10000)
    kp1, des1 = feat.detectAndCompute(img1,None)
    kp2, des2 = feat.detectAndCompute(img2,None)
    print '#features of image1:', len(kp1)
    print '#features of image2:', len(kp2)

    bf = cv.BFMatcher()
    matches = bf.knnMatch(des1,des2,k=2)
    print '#matchers after KNN:', len(matches)

    good = []
    if ratio_test:
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                good.append([m, n])
        print '#matchers after ratio test:', len(good)
    else:
        good = matches

    if log_file != '':
        with open(log_file, 'w') as fout:
            fout.write('{}\n'.format(len(matches)))
            for m,n in matches:
                index1 = m.queryIdx
                index2 = m.trainIdx
                coord1 = kp1[index1].pt
                coord2 = kp2[index2].pt
                ratio = m.distance / n.distance # ratio \in (0, 1)
                prob = 0.5#1 - ratio                # The smaller the ratio, the higher the probability
                fout.write('%f %f %f %f %f %f %f\n' % (coord1[0], coord1[1], 0.0, coord2[0], coord2[1], 0.0, prob))

    # img3 = cv.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    img3 = drawMatches(img1, kp1, img2, kp2, good)
    img3 = cv.resize(img3, (img3.shape[1] / 10, img3.shape[0] / 10))
    cv.imwrite("/home/larry/matches.png", img3)
    img3 = cv.cvtColor(img3, cv.COLOR_BGR2RGB)
    plt.imshow(img3)
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('image1', type=str, help="Image file1")
    parser.add_argument('image2', type=str, help="Image file2")
    parser.add_argument('--output', type=str, default='', help="Log file")
    args = parser.parse_args()

    feature_match(args.image1, args.image2, ratio_test=True, log_file=args.output)
