import time
import logging
import sys
import os

def setup_logger():
    timestr = time.strftime("%Y%m%d-%H%M%S")
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s | %(levelname)s | %(message)s',
                                  '%m-%d-%Y %H:%M:%S')

    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setLevel(logging.DEBUG)
    stdout_handler.setFormatter(formatter)

    if not os.path.exists("logs"):
        os.mkdir("logs")
    file_handler = logging.FileHandler(
        "logs/second_stage_{}.log".format(timestr))
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)

    logger.addHandler(file_handler)
    logger.addHandler(stdout_handler)
    return logger
