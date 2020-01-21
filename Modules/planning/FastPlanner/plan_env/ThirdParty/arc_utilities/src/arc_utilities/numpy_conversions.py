import numpy as np

def ListPointsToNpArray(points, transform=None):
    """
    :param points: Anything that is itterable, with each element having x, y, and z elements
    :type points: :class:`list[geometry_msgs.msgs.Point]`
    :param transform:
    :type transform: :class:`np.array`
    :return:
    """
    arr = np.empty(shape=(3, len(points)))
    if transform is not None:
        for ind in range(len(points)):
            point = transform.dot([points[ind].x, points[ind].y, points[ind].z, 1])
            arr[:, ind] = point[0:3]
    else:
        for ind in range(len(points)):
            arr[:, ind] = [points[ind].x, points[ind].y, points[ind].z]
    return arr
