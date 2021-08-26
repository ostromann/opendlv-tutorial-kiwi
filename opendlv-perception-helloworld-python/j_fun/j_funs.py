import numpy as np
import cv2



def get_cones_imgs(img):

    DILATIONS = 1
    EROSIONS = 1

    img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower = np.array([110, 50, 50], np.uint8)  # 10, 100, 100
    upper = np.array([130, 255, 255], np.uint8)  # 30, 255, 255
    img_blue = cv2.inRange(img2, lower, upper)

    kernel = np.ones((5, 5), 'uint8')
    img_blue = cv2.dilate(img_blue, kernel, iterations=DILATIONS)
    img_blue = cv2.erode(img_blue, kernel, iterations=EROSIONS)

    lower = np.array([10, 100, 100], np.uint8)  # 10, 100, 100
    upper = np.array([30, 255, 255], np.uint8)  # 30, 255, 255
    img_yellow = cv2.inRange(img2, lower, upper)

    kernel = np.ones((5, 5), 'uint8')
    img_yellow = cv2.dilate(img_yellow, kernel, iterations=DILATIONS)
    img_yellow = cv2.erode(img_yellow, kernel, iterations=EROSIONS)


    return img_blue, img_yellow


def get_contours(thresh, type):
    EPSILON = 1
    contours, h = cv2.findContours(thresh, 1, 2)  # 1, 2

    # contours_poly = [None] * len(contours)
    # boundRects = [None] * len(contours)
    # centers = [None] * len(contours)
    # radiuses = [None] * len(contours)


    contours_poly = []
    boundRects = []
    centers = []
    radiuses = []


    for i, c in enumerate(contours):

        contour = cv2.approxPolyDP(c, EPSILON, True)
        rect = cv2.boundingRect(contour)
        center, radius = cv2.minEnclosingCircle(contour)

        if radius < 10 or radius > 60:
            pass
        elif type == 'b' and center[0] < 200 or type == 'y' and center[0] > 1000:
            pass
        else:
            contours_poly.append(contour)
            boundRects.append(rect)
            centers.append(center)
            radiuses.append(radius)
            # contours_poly[i] = contour
            # boundRects[i] = rect
            # centers[i] = center
            # radiuses[i] = radius

        # contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        # boundRect[i] = cv2.boundingRect(contours_poly[i])
        # centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])

    xs_center = []
    ys_center = []

    if len(centers) > 0:
        for i, c in enumerate(centers):
            xs_center.append(c[0])
            ys_center.append(c[1])

    contours_di = {"contours_poly": contours_poly,
                  "boundRect": boundRects,
                  "centers": centers,
                   "radius": radiuses}

    #TEMP===========
    # contours, h = cv2.findContours(thresh, 1, 2)
    # contours_poly = [None] * len(contours)
    # boundRect = [None] * len(contours)
    # centers = [None] * len(contours)
    # radius = [None] * len(contours)
    #
    # for i, c in enumerate(contours):
    #     contours_poly[i] = cv2.approxPolyDP(c, 3, True)
    #     boundRect[i] = cv2.boundingRect(contours_poly[i])
    #     centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])
    #
    # # contours_di = {"contours_poly": contours_poly,
    # #                "boundRect": boundRect,
    # #                "centers": centers,
    # #                "radius": radius}
    #=======

    return contours_di, xs_center, ys_center


def weighted_mean_centers(Y_WEIGHTING, xs_center, ys_center):

    weights = []
    ys_weighted = []
    xs_weighted = []

    for i, y in enumerate(ys_center):
        weights.append(Y_WEIGHTING[int(y)])
    weights_sum = np.sum(weights)

    for i in range(len(ys_center)):
        ys_weighted.append(ys_center[i] * weights[i])
        xs_weighted.append(xs_center[i] * weights[i])

    x_mid = int(np.sum(xs_weighted) / weights_sum)
    y_mid = int(np.sum(ys_weighted) / weights_sum)

    # aa = (int(np.mean(xs_center)), int(np.mean(ys_center)))
    # return aa
    # return (y_mid, x_mid)

    return (x_mid, y_mid)


def draw_s(img, contours, color):
    num_contours = len(contours['contours_poly'])

    for i in range(num_contours):
        # color = (255, 0, 0)  # BGR

        try:
            cv2.drawContours(img, contours['contours_poly'], i, color)
            cv2.rectangle(img, (int(contours['boundRect'][i][0]), int(contours['boundRect'][i][1])),
                         (int(contours['boundRect'][i][0] + contours['boundRect'][i][2]),
                          int(contours['boundRect'][i][1] + contours['boundRect'][i][3])), color, 2)
            cv2.circle(img, (int(contours['centers'][i][0]),
                             int(contours['centers'][i][1])),
                       int(contours['radius'][i]), color, 2)
        except:
            pass
        # cv2.drawContours(img_blue, contours_poly, i, color)
        # cv2.rectangle(img_blue, (int(boundRect[i][0]), int(boundRect[i][1])), (int(boundRect[i][0] + boundRect[i][2]),
        #                                                                        int(boundRect[i][1] + boundRect[i][3])), color, 2)
        # cv2.circle(img_blue, (int(centers[i][0]),
        #                       int(centers[i][1])),
        #            int(radius[i]), color, 2)





