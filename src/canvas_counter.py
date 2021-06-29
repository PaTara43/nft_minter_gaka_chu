#!/usr/bin/python3

import rospy


def canvas_counter(path: str) -> bool:
    """

    :param path: Path to file with numbers of canvases
    :return: True, if there are no more canvases. False, if there are a few canvases.
    """
    try:
        f = open(path, "r+")
        number = int(f.read())
        f.seek(0)
        f.truncate()
        rospy.loginfo(f"Previous number of canvases: {number}")
    except Exception as e:
        rospy.loginfo("can't open the file!")
        rospy.loginfo(e)
        exit()
    number -= 1
    rospy.loginfo(f"Current number of canvases: {number}")
    if number == 0:
        f.write("3")
        f.close()
        rospy.loginfo(f"Need to order canvases.")
        return True
    else:
        f.write(str(number))
        f.close()
        rospy.loginfo(f"Can continue drawing.")
        return False
