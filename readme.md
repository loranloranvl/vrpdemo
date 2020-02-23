# Class VRP
## 参数说明
* distance_matrix 方阵
* demands 列表 各个配送点的需求，配送点次序应当与distance_matrix保持一致
* depot 整数 配送中心的下标，默认为0，即distance_matrix中第一个出现的点
* num_vehicles 整数 车辆数量，若不提供则由算法自动产生
* vehicle_capacities 列表 车辆容量，若不提供则由算法自动生成。
    * 当提供vehicle_capacities但是不提供num_vehicles时，vehicle_capacities内的元素不应当重复，表示可供选择的车辆
    * 当同时提供两个参数时，应当保证len(vehicle_capacities)==num_vehicles
* time_limit 整数（秒） 超过这个时间则叫停算法，返回结果
* strategy 字符串 指定具体启发式算法 可选：
    * AUTOMATIC	
    * GREEDY_DESCENT	
    * GUIDED_LOCAL_SEARCH	
    * SIMULATED_ANNEALING	
    * TABU_SEARCH
    * OBJECTIVE_TABU_SEARCH
    
class VRP中只做了少数必要的参数校验，前后端务必要做严格且一致的参数校验及报错提示。后端不可寄希望前端的参数都是校验过的。

# class TestVRP
运行测试案例后可在自动生成的routing.png查看结果