import cv2 as cv
import numpy as np
import math
import open3d as o3d


def main():
    # desired size
    length, width, height = 30, 30, 5

    # offset, (0, 0, 0) for center align
    # offset = (0, 0, -0.5)
    offset = (0, 0, 0)

    # thresh to cutoff
    # height_thresh = -0.1
    resolution = 0.2

    # in and out path
    img_path = "../assets/models/valley_heightmap/materials/textures/heightmap.png"
    out_path = "../assets/outdoor.pcd"
    
    # load heightmap
    img = cv.imread(img_path, cv.IMREAD_GRAYSCALE)
    img_h, img_w = img.shape

    assert length % 2 == 0, "length is not even"
    assert width % 2 == 0, "width is not even"

    # rotate
    center = (img_h // 2, img_w // 2)
    m = cv.getRotationMatrix2D(center, -90, 1)
    img = cv.warpAffine(img, m, (img_h, img_w))

    points = []
    for x in np.arange(0, length, resolution):
        for y in np.arange(0, width, resolution):
            i = math.floor(x / length * img_h)
            j = math.floor(y / width * img_w)
            z = img[i, j] / 255 * height
            # if z + offset[2] < height_thresh:
            #     continue

            point = np.array([x - length / 2 + offset[0], y - width / 2 + offset[1], z + offset[2]])
            points.append(point)

            # check surrounding, if there is a lower z, completion along z
            # z_min = z
            # for x_off in range(-1, 3, 2):
            #     for y_off in range(-1, 3, 2):
            #         x_n = x + x_off * resolution
            #         y_n = y + y_off * resolution
            #         if x_n < 0 or x_n >= length or y_n < 0 or y_n >= width:
            #             continue
            #         i_n = math.floor(x_n / length * img_h)
            #         j_n = math.floor(y_n / width * img_w)
            #         z_n = img[i_n, j_n] / 255 * height
            #         if z_n < z_min:
            #             z_min = z_n

            # from 0 to z, not checking surrounding now
            for z_t in np.arange(0, z, resolution):
                point = np.array([x - length / 2 + offset[0], y - width / 2 + + offset[1], z_t + offset[2]])
                points.append(point)

    points = np.array(points)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    print(point_cloud)

    # default False, saved as Binarty, otherwise ASICC
    o3d.io.write_point_cloud(out_path, point_cloud)
    print("pcd saved! {}".format(out_path))

    # plot
    o3d.open3d.visualization.draw_geometries([point_cloud])


if __name__ == '__main__':
    main()
