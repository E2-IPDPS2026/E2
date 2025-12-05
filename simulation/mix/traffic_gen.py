#!/usr/bin/env python3
"""
生成两个 trace 文件（造成拥塞的流）：
 1) datamining_congestion.txt  -- 每个源主机一个大流（容易造成持续拥塞）
 2) websearch_congestion.txt  -- 每个源主机多个小流（短时高并发，产生短暂拥塞）

输出格式（每行）:
 src dst priority port flowsize(B) start_time(s)

第一行为流条目总数（不计首行）。
"""

import random
import argparse
from datetime import datetime

def gen_datamining(n_hosts=1024, n_dst=32,
                   flowsize_bytes=200*1024*1024,
                   priority=3, port=100,
                   start_time_range=(2.0, 5.0),
                   seed=42):
    """
    每个源产生 1 个大流，目的在 n_dst 个热点中随机选择，
    start_time 在 start_time_range 内随机分布（产生时间重叠 -> 拥塞）。
    返回 list of lines (strings).
    """
    random.seed(seed)
    lines = []
    for src in range(n_hosts):
        dst = random.randint(0, n_dst-1)  # hotspot
        st = 2
        lines.append(f"{src} {dst} {priority} {port} {flowsize_bytes} {st:.9f}")
    return lines

def gen_websearch(n_hosts=1024, n_dst=32,
                   per_host=5,
                   flowsize_bytes=50*1024,
                   priority=3,
                   ports=(101,102,103,104,105),
                   start_time_range=(2.0, 10.0),
                   seed=12345):
    """
    每个源产生 per_host 条短流，目的在 n_dst 个热点中随机选择，
    start_time 在 start_time_range 内随机分布。
    """
    random.seed(seed)
    lines = []
    n_ports = len(ports)
    for src in range(n_hosts):
        for k in range(per_host):
            dst = random.randint(0, n_dst-1)
            port = ports[k % n_ports]
            st = 2
            lines.append(f"{src} {dst} {priority} {port} {flowsize_bytes} {st:.9f}")
    return lines

def write_trace(filename, lines):
    with open(filename, "w") as f:
        f.write(str(len(lines)) + "\n")
        for l in lines:
            f.write(l + "\n")
    print(f"Wrote {len(lines)} flows to {filename}")

def main():
    parser = argparse.ArgumentParser(description="Generate congestion traces for 1024 hosts.")
    parser.add_argument("--n_hosts", type=int, default=1024, help="Number of hosts (default 1024)")
    parser.add_argument("--n_dst", type=int, default=32, help="Number of hotspot destination hosts (default 32)")
    parser.add_argument("--web_per_host", type=int, default=5, help="Websearch flows per host (default 5)")
    parser.add_argument("--out_dm", default="traffic_AI_datamining.txt", help="Output file for datamining flows")
    parser.add_argument("--out_ws", default="traffic_AI_websearch.txt", help="Output file for websearch flows")
    parser.add_argument("--seed", type=int, default=20251008, help="Random seed (default 20251008)")
    args = parser.parse_args()

    # Parameters (可按需调整)
    N_HOSTS = args.n_hosts
    N_DST = args.n_dst
    WEB_PER_HOST = args.web_per_host

    # Data mining: large flows
    DM_FLOWSIZE = 200 * 1024 * 1024   # 200 MB
    DM_PRIORITY = 3
    DM_PORT = 100
    DM_START_RANGE = (2.0, 5.0)       # 集中开始，易拥塞

    # Websearch: small short flows
    WS_FLOWSIZE = 50 * 1024           # 50 KB
    WS_PRIORITY = 3
    WS_PORTS = (101,102,103,104,105)
    WS_START_RANGE = (2.0, 10.0)

    now = datetime.now().isoformat(sep=' ', timespec='seconds')
    print(f"Generating traces at {now}")
    print(f"Hosts: {N_HOSTS}, Hotspots: {N_DST}, web flows per host: {WEB_PER_HOST}")
    print("Generating datamining flows...")
    dm_lines = gen_datamining(n_hosts=N_HOSTS, n_dst=N_DST,
                              flowsize_bytes=DM_FLOWSIZE,
                              priority=DM_PRIORITY, port=DM_PORT,
                              start_time_range=DM_START_RANGE,
                              seed=args.seed)

    print("Generating websearch flows...")
    ws_lines = gen_websearch(n_hosts=N_HOSTS, n_dst=N_DST,
                             per_host=WEB_PER_HOST,
                             flowsize_bytes=WS_FLOWSIZE,
                             priority=WS_PRIORITY,
                             ports=WS_PORTS,
                             start_time_range=WS_START_RANGE,
                             seed=args.seed + 1)

    write_trace(args.out_dm, dm_lines)
    write_trace(args.out_ws, ws_lines)
    print("Done.")

if __name__ == "__main__":
    main()
