import os
import time
import logging
import numpy as np
import pandas as pd
from pathlib import Path
from dotenv import load_dotenv

load_dotenv()

from kiva_sim.orders import init_orders, Order
from kiva_sim.simulation import simulation


# data = {
#     "AGV num": [],
#     "order num": [],
#     "time step": [],  # 效率指标
#     "revenue per hour": [],  # 利润指标
#     "agv utility": [],  # 利用率指标
#     "total net revenue": [],
#     "revenue": [],
#     "AGV cost": [],
#     "charging cost": [],
#     "worker cost": [],
#     "time step ub": [],  # 各指标上界
#     "revenue per hour ub": [],
#     "agv utility ub": [],
#     "total net revenue ub": [],
#     "revenue ub": [],
#     "AGV cost ub": [],
#     "charging cost ub": [],
#     "worker cost ub": [],
#     "time step lb": [],  # 各指标下界
#     "revenue per hour lb": [],
#     "agv utility lb": [],
#     "total net revenue lb": [],
#     "revenue lb": [],
#     "AGV cost lb": [],
#     "charging cost lb": [],
#     "worker cost lb": [],
# }
# summary = pd.DataFrame(
#     data,
# )

# 注释部分为数值实验
# ORDER_NUM = 7  # 订单数量
# REPEAT = 5  # 重复实验次数
# AGV_start_num = 16
# AGV_end_num = 20
# for i in range(AGV_start_num, AGV_end_num + 1):
#     summary_repeat = []
#     for repeat in range(REPEAT):
#         order_list = init_orders(seed=round(time.time()%1 * 100000), numOfOrders=ORDER_NUM)  # 生成订单
#         summary_repeat.append(simulation(seed=round(time.time()%1 * 100000), AGV_NUM=i, ORDER_NUM=ORDER_NUM, order_list=order_list))
#     mean = np.mean(summary_repeat, axis=0)
#     standard_error = np.sqrt(np.var(summary_repeat, axis=0) / REPEAT)
#     upper_bound = mean[2:] + 1.96 * standard_error[2:]
#     lower_bound = mean[2:] - 1.96 * standard_error[2:]
#     summary.loc[i] = np.concatenate((mean, upper_bound, lower_bound)).round(4)
# summary.to_excel('map1 AGV%d-%d.xlsx'%(AGV_start_num, AGV_end_num))

if __name__ == "__main__":
    log_folder = Path(__file__).parent / "logs"
    log_folder.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    log_file = log_folder / f"simulation_{timestamp}.log"
    log = logging.basicConfig(
        filename=log_file,
        filemode="w",
        format="%(asctime)s | %(levelname)-8s | %(filename)s:%(lineno)d | %(message)s",
        datefmt="%Y-%m-%d-%H:%M:%S",
        level=logging.INFO,
    )

    order_num = 2  # 订单数量

    simulation(
        agv_num=12,
        order_num=order_num,
        interval=200,
        show=True,
        save_fig=False,
        heat_map=False,
        random_seed=100,
        astar_max_iter=2000,
    )
