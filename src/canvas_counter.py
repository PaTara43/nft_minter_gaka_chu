#!/usr/bin/python3

import logging


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
        logging.debug(f"Previous number of canvases: {number}")
    except Exception as e:
        logging.error("can't open the file!")
        logging.error(e)
        exit()
    number -= 1
    logging.debug(f"Current number of canvases: {number}")
    if number == 0:
        f.write("3")
        f.close()
        logging.debug(f"Need to order canvases.")
        return True
    else:
        f.write(str(number))
        f.close()
        logging.debug(f"Can continue drawing.")
        return False
