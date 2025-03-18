# FuAR
[demo videos](https://sites.google.com/view/lsmfuar)
## Prerequisites

1. Ubuntu 22.04 LTS
2. [Docker CE](https://docs.docker.com/engine/install/ubuntu/)
3. [Python 3.11](https://www.python.org/downloads/release/python-3110/)
4. Baidu Apollo v7.0.0(https://github.com/ApolloAuto/apollo)
5. SORA-SVL(https://github.com/YuqiHuai/SORA-SVL)

## Structure of FuAR's Approach

```
The structure of folder "Approach" is as follows:
approach         
│── combined_reports_strict_rule                    fusion judgment 
│── information_ex_results_pro                      information extraction of accident reports
│── runtime_scenaio_data                            runtime data of test apollo                                               
├── generate_conbinable_graph.py                    directed graph         
├── generate_conbined_reports.py                    generate fused reports         
├── informationexact.py                             exract information                 
├── monitor_ego_and_npcs_data.py                    monitor ego and npcs                         
├── simultate_report.py                             test apollo in fused scenarios                   
├── solvewaypoints.py                               model trajectory
└── utils_lsm.py                                    basic methods     
```

##  Conda Environment Setup
```
pip install -r requirements.txt
```

##  Navigate to Apollo scripts directory
```
cd apollo/docker/scripts/
```

##  Start Apollo containers
```
bash dev_start.sh
```

##  Enter development container
```
bash dev_into.sh
```

## Start bootstrap and bridge
```
bash bootstrap.sh
bash bridge.sh
```

## Start SORA-SVL
Refer to [https://github.com/YuqiHuai/SORA-SVL](https://github.com/YuqiHuai/SORA-SVL)

## Start automation test
```
python /home/yourusername/FuAR/src/approach/simulate_report.py
```
