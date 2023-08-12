# from __future__ import print_function
import cv2
import numpy as np
import argparse
import matplotlib.pyplot as plt
import shapely
from shapely import geometry
from shapely import affinity
from shapely import ops

src = None
np_src = None
np_unk = None
np_free = None
np_obstacle = None
np_result = None
erosion_size = 0
max_elem = 2
max_kernel_size = 21
title_trackbar_element_shape = 'Element:\n 0: Rect \n 1: Cross \n 2: Ellipse'
title_trackbar_kernel_size = 'Kernel size:\n 2n +1'
title_erosion_window = 'Erosion Demo'
title_dilation_window = 'Dilation Demo'
unk_window = 'UNK window'
obst_window = 'OBSTACLE window'
free_window = 'FREE window'
np_obstacle_margin_window = 'np_obstacle_margin'
np_free_filled_window = 'np_free_filled'
np_unk_cleaned_window = 'np_unk_cleaned'
np_result_window = 'RESULT'


def get_shapely_poly(polygons, contours, hierarchy, poly_id, external, internals):
    while poly_id != -1:
        contour = contours[poly_id].squeeze(axis=1)
        if len(contour) >= 3:
            first_child_id = hierarchy[poly_id][2]
            children = [] if external else None
            get_shapely_poly(polygons, contours, hierarchy, first_child_id, not external, children)

            if external:
                polygon = geometry.Polygon(contour, holes=children)
                polygons.append(polygon)
            else:
                internals.append(contour)

        poly_id = hierarchy[poly_id][0]


def main(image):
    global src
    global np_unk
    global np_free
    global np_obstacle


    src = cv2.imread(cv2.samples.findFile(image), cv2.IMREAD_UNCHANGED)
    if src is None:
        print('Could not open or find the image: ', image)
        exit(0)
    # np_src = src.astype('int8')
    width, height = np.shape(src)
    np_src = np.copy(src)
    np_unk = np.zeros_like(src, dtype=np.uint8)
    np_unk.fill(254)
    np_free = np.zeros_like(src, dtype=np.uint8)
    np_free.fill(254)
    np_obstacle = np.zeros_like(src, dtype=np.uint8)
    np_obstacle.fill(254)
    # for line in np_src:
    #     for col in line:
    #         # print((255 - col) / 255.0)
    #         print(col)

    np_unk = np.where(src == 205, 0, np_unk)
    np_obstacle = np.where(src == 0, 0, np_obstacle)
    np_free = np.where(src == 254, 0, np_free)


    # Remove unknown noise
    shape_size = 2
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                       (2 * shape_size + 1, 2 * shape_size + 1),
                                       (shape_size, shape_size))
    np_unk_cleaned = cv2.dilate(np_unk, kernel, iterations=1)
    # Unknown as obstacle
    np_obstacle = np.where(np_unk_cleaned == 0, np_unk_cleaned, np_obstacle)

    # Obstacle margin
    shape_size = 6
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                       (2 * shape_size + 1, 2 * shape_size + 1),
                                       (shape_size, shape_size))
    np_obstacle_margin = cv2.erode(np_obstacle, kernel, iterations=1)

    # Fill holes in free
    shape_size = 3
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                       (2 * shape_size + 1, 2 * shape_size + 1),
                                       (shape_size, shape_size))
    np_free_filled = cv2.erode(np_free, kernel, iterations=1)


    image_copy = src.copy()
    plt.imshow(image_copy)
    plt.show()
    for i in range(0, 25):
        contours, hierarchy = cv2.findContours(np_obstacle_margin, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if hierarchy is None:
            break

        polygons = []
        hierarchy = hierarchy[0]
        get_shapely_poly(polygons, contours, hierarchy, 0, True, [])
        simplify_limit = 3
        for poly in polygons:
            poly = poly.simplify(simplify_limit, preserve_topology=True)
            print(poly.area)
            points = [[x, y] for x, y in zip(*poly.exterior.coords.xy)]
            image_copy = cv2.polylines(image_copy, pts=np.array([points]).astype(np.int32),
                                       isClosed=True, color=80, thickness=1)

            for inner in poly.interiors:
                inner = inner.simplify(simplify_limit, preserve_topology=True)
                print(inner.area)
                points = [[x, y] for x, y in zip(*inner.coords.xy)]
                image_copy = cv2.polylines(image_copy, pts=np.array([points]).astype(np.int32),
                                           isClosed=True, color=190, thickness=1)

            plt.imshow(image_copy)
            plt.show()



        cv2.namedWindow(np_result_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(np_result_window, height, width)
        cv2.imshow(np_result_window, image_copy)
        cv2.waitKey()

        # Obstacle margin
        shape_size = 4
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (2 * shape_size + 1, 2 * shape_size + 1),
                                           (shape_size, shape_size))
        np_obstacle_margin = cv2.erode(np_obstacle_margin, kernel, iterations=1)



    #
    # cv2.namedWindow(np_obstacle_margin_window, cv2.WINDOW_NORMAL)
    # cv2.resizeWindow(np_obstacle_margin_window, height, width)
    # cv2.imshow(np_obstacle_margin_window, np_obstacle_margin)

    # cv2.namedWindow(unk_window, cv2.WINDOW_NORMAL)
    # cv2.resizeWindow(unk_window, height, width)
    # cv2.imshow(unk_window, np_unk)
    #
    # cv2.namedWindow(obst_window, cv2.WINDOW_NORMAL)
    # cv2.resizeWindow(obst_window, height, width)
    # cv2.imshow(obst_window, np_obstacle)
    #
    # cv2.namedWindow(free_window, cv2.WINDOW_NORMAL)
    # cv2.resizeWindow(free_window, height, width)
    # cv2.imshow(free_window, np_free)




    # cv2.namedWindow(np_free_filled_window, cv2.WINDOW_NORMAL)
    # cv2.resizeWindow(np_free_filled_window, height, width)
    # cv2.imshow(np_free_filled_window, np_free_filled)
    #
    # cv2.namedWindow(np_unk_cleaned_window, cv2.WINDOW_NORMAL)
    # cv2.resizeWindow(np_unk_cleaned_window, height, width)
    # cv2.imshow(np_unk_cleaned_window, np_unk_cleaned)
    #
    #
    # cv2.namedWindow(title_erosion_window, cv2.WINDOW_NORMAL)
    # cv2.resizeWindow(title_erosion_window, 500, 700)
    # cv2.createTrackbar(title_trackbar_element_shape, title_erosion_window, 0, max_elem, erosion)
    # cv2.createTrackbar(title_trackbar_kernel_size, title_erosion_window, 0, max_kernel_size, erosion)
    # cv2.namedWindow(title_dilation_window, cv2.WINDOW_NORMAL)
    # cv2.resizeWindow(title_dilation_window, 500, 700)
    # cv2.createTrackbar(title_trackbar_element_shape, title_dilation_window, 0, max_elem, dilatation)
    # cv2.createTrackbar(title_trackbar_kernel_size, title_dilation_window, 0, max_kernel_size,
    #                   dilatation)
    # erosion(0)
    # dilatation(0)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Code for Eroding and Dilating tutorial.')
    parser.add_argument('--input', help='Path to input image.', default='gridmap.pgm')
    args = parser.parse_args()
    main(args.input)
