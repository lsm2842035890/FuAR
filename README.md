# FuAR

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
├── base_map.txt                                    base map of apollo                
├── generate_conbinable_graph.py                    directed graph         
├── generate_conbined_reports.py                    generate fused reports         
├── informationexact.py                             exract information                 
├── monitor_ego_and_npcs_data.py                    monitor ego and npcs                         
├── simultate_report.py                             test apollo in fused scenarios                   
├── solvewaypoints.py                               model trajectory
└── utils_lsm.py                                    basic methods     
```


## Citing
