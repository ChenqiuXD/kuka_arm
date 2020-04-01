from cv2 import aruco
import cv2
import matplotlib.pyplot as plt
import matplotlib as mpl

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# for i in range(1, nx*ny+1):
#     ax = fig.add_subplot(ny,nx, i)
#     img = aruco.drawMarker(aruco_dict,i, 700)
#     plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
    # ax.axis("off")
img = aruco.drawMarker(aruco_dict, 1, 700)
cv2.imwrite('pic.png', img)
# cv2.imshow('img', img)
# cv2.waitKey(0)
# plt.imshow(img, cmap = mpl.cm.gray, interpolation="nearest")
# plt.savefig("pic")
# plt.show()