#!/usr/bin/env python
import logging
import colorlog


def get_logger(logger_name, log_level=logging.INFO):

    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.DEBUG)

    # formatter = logging.Formatter(
    #     "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    # )
    # formatter = logging.Formatter('%(name)s - %(levelname)s - %(message)s')

    # 定义颜色输出格式
    color_formatter = colorlog.ColoredFormatter(
        fmt="%(log_color)s[%(asctime)s.%(msecs)03d] %(filename)s -> %(funcName)s line:%(lineno)d [%(levelname)s] : %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        log_colors={
            "DEBUG": "white",  # cyan white
            "INFO": "green",
            "WARNING": "yellow",
            "ERROR": "red",
            "CRITICAL": "bold_red",  #'red,bg_white',
        },
    )

    # 将颜色输出格式添加到控制台日志处理器
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(color_formatter)
    console_handler.setLevel(log_level)

    # fileHandler = logging.FileHandler(f"{output}/metabcc-lr.log")
    # fileHandler.setLevel(logging.DEBUG)
    # fileHandler.setFormatter(formatter)

    # logger.addHandler(fileHandler)
    logger.addHandler(console_handler)
    return logger
