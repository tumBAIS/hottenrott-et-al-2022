//------------------------------------------------------------------------------------------------------------//
/* Initialize available routes and available task location combinations for all vehicles at beginning of node. */
//------------------------------------------------------------------------------------------------------------//
using namespace std;
//------------------------------------------------------------------------------------------------------------//
// Type definitions
typedef vector<int> VecI;
typedef vector<bool> VecB;
typedef vector<double> VecD;
typedef vector<string> VecS;
typedef vector<size_t> VecT;
//------------------------------------------------------------------------------------------------------------//
/* Preprocess node.*/
void preprocess_node(vector<Model> &models, vector<Vehicle> &vehicles, vector<Location> &locations,
        vector<vector<vector<VecI>>> &blockings, vector<vector<vector<VecI>>> &available_blockings,
        Node &parent_node, int &number_tasks, int &number_locations, int &ub, int &horizon) {
    // determine available routes
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        // reset available routes
        (*v_it).Available_routes = {};
        Model model(*(models.begin() + (*v_it).Type));
        for (auto r_it = (*v_it).All_routes.begin(); r_it != (*v_it).All_routes.end(); ++r_it) {
            // ensure that route is shorter than ub
            if ((*v_it).Route_scts[(*r_it)] >= ub)
                continue;
            // ensure that route fulfills fixed task locations of scenario and of node
            bool feasible(true);
            for (auto x_it = (*v_it).Route_details[(*r_it)].begin(); x_it != (*v_it).Route_details[(*r_it)].end(); ++x_it) {
                if (find(model.Scenario_task_location_combinations[(*x_it)[0]].begin(), model.Scenario_task_location_combinations[(*x_it)[0]].end(), (*x_it)[1]) == model.Scenario_task_location_combinations[(*x_it)[0]].end()) {
                    feasible = false;
                    break;
                }
                for (int t = 0; t != number_tasks; ++t) {
                    if (parent_node.Fixed_vehicle_location_tasks[(*v_it).Id][(*x_it)[0]][t] != -1 && (int) model.Possible_task_location_combinations[(*x_it)[0]][(*x_it)[1]][t] != parent_node.Fixed_vehicle_location_tasks[(*v_it).Id][(*x_it)[0]][t]) {
                        feasible = false;
                        break;
                    }
                }
                if (!feasible)
                    break;
            }
            if (!feasible) {
                continue;
            }
            // ensure that route fufills precedences of scenario
            VecI start_times(number_tasks, 0); // derive start times of operations that include a certain task
            for (auto x_it = (*v_it).Route_details[(*r_it)].begin(); x_it != (*v_it).Route_details[(*r_it)].end(); ++x_it) {
                for (auto t = model.Possible_task_location_combinations_list[(*x_it)[0]][(*x_it)[1]].begin(); t != model.Possible_task_location_combinations_list[(*x_it)[0]][(*x_it)[1]].end(); ++t) {
                    start_times[*t] = (*x_it)[2];
                }
            }
            for (int t1 = 0; t1 != number_tasks; ++t1) {
                for (int t2 = 0; t2 != number_tasks; ++t2) {
                    if (model.Scenario_all_successors[t1][t2]) {
                        if (start_times[t1] > start_times[t2]) {
                            feasible = false;
                            break;
                        }
                    }
                }
                if (!feasible)
                    break;
            }
            if (!feasible) {
                continue;
            }
            // ensure that route fulfilles enforced and forbidden location times of node
            for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
                for (auto it = parent_node.Enforced_vehicle_location_times[(*v_it).Id][(*l_it).Id].begin(); it != parent_node.Enforced_vehicle_location_times[(*v_it).Id][(*l_it).Id].end(); ++it) {
                    if ((*v_it).Route_start_times[(*r_it)][(*l_it).Id] > (*it) || (*v_it).Route_end_times[(*r_it)][(*l_it).Id] - 1 < (*it)) {
                        feasible = false;
                        break;
                    }
                }
                if (!feasible)
                    break;
                for (auto it = parent_node.Forbidden_vehicle_location_times[(*v_it).Id][(*l_it).Id].begin(); it != parent_node.Forbidden_vehicle_location_times[(*v_it).Id][(*l_it).Id].end(); ++it) {
                    if ((*v_it).Route_start_times[(*r_it)][(*l_it).Id] <= (*it) && (*v_it).Route_end_times[(*r_it)][(*l_it).Id] - 1 >= (*it)) {
                        feasible = false;
                        break;
                    }
                }
                if (!feasible)
                    break;
            }
            if (!feasible) {
                continue;
            }
            // ensure that task time corridors are fulfilled
            for (int t = 0; t != number_tasks; ++t) {
                if (!model.Tasks_per_model[t])
                    continue;
                if (start_times[t] < parent_node.Task_start_times_lower_bounds[(*v_it).Id][t] || start_times[t] > parent_node.Task_start_times_upper_bounds[(*v_it).Id][t]) {
                    feasible = false;
                    break;
                }
            }
            if (!feasible) {
                continue;
            }
            (*v_it).Available_routes.push_back((*r_it));
        }
    }
    // determine available task-location combinations
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        // reset available task-location combinations
        VecI dummy({});
        vector<VecI> reset_ltc(number_locations, dummy);
        (*v_it).Available_task_location_combinations = reset_ltc;
        Model model(*(models.begin() + (*v_it).Type));
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            for (size_t c_index = 0; c_index != model.Possible_task_location_combinations[(*l_it).Id].size(); ++c_index) {
                int c((int) c_index);
                if (find(model.Scenario_task_location_combinations[(*l_it).Id].begin(), model.Scenario_task_location_combinations[(*l_it).Id].end(), c) == model.Scenario_task_location_combinations[(*l_it).Id].end())
                    continue;
                bool feasible_c(true);
                for (int t = 0; t != number_tasks; ++t) {
                    if (parent_node.Fixed_vehicle_location_tasks[(*v_it).Id][(*l_it).Id][t] != -1 && (int) model.Possible_task_location_combinations[(*l_it).Id][c_index][t] != parent_node.Fixed_vehicle_location_tasks[(*v_it).Id][(*l_it).Id][t]) {
                        feasible_c = false;
                        break;
                    }
                }
                if (!feasible_c)
                    continue;
                (*v_it).Available_task_location_combinations[(*l_it).Id].push_back(c);
            }
        }
    }
    // determine available blockings
    for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
        vector<vector<VecI>> available_blockings_l;
        for (int z = 0; z != horizon; ++z) {
            vector<VecI> available_blockings_lz;
            for (auto it = blockings[(*l_it).Id][z].begin(); it != blockings[(*l_it).Id][z].end(); ++it) {
                if (find(vehicles[(*it)[0]].Available_routes.begin(), vehicles[(*it)[0]].Available_routes.end(), (*it)[1]) != vehicles[(*it)[0]].Available_routes.end()) {
                    available_blockings_lz.push_back((*it));
                }
            }
            available_blockings_l.push_back(available_blockings_lz);
        }
        available_blockings.push_back(available_blockings_l);
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Check integrality of master problem solution. */
void check_integrality(bool &node_is_integral, vector<Vehicle> &vehicles, vector<vector<GRBVar>> &use_route) {
    node_is_integral = true;
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        for (size_t r_index = 0; r_index != (*v_it).Available_routes.size(); ++r_index) {
            if (use_route[(*v_it).Id][r_index].get(GRB_DoubleAttr_X) >= 0.00001 && use_route[(*v_it).Id][r_index].get(GRB_DoubleAttr_X) <= 0.99999) {
                node_is_integral = false;
                break;
            }
        }
        if (!node_is_integral)
            break;
    }
}
//------------------------------------------------------------------------------------------------------------//
void update_best(int &ub, double &node_lb, GRBVar &segment_cycle_time, VecI &best_sol, vector<Vehicle> &vehicles, vector<vector<GRBVar>> &use_route) {
    ub = (int) round(node_lb);
    cout << "NEW BEST: " << ub << endl;
    VecI new_best_solution;
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        for (size_t r_index = 0; r_index != (*v_it).Available_routes.size(); ++r_index) {
            if (use_route[(*v_it).Id][r_index].get(GRB_DoubleAttr_X) > 0.9999) {
                new_best_solution.push_back((*v_it).Available_routes[r_index]);
            }
        }
    }
    best_sol = new_best_solution;
}
//------------------------------------------------------------------------------------------------------------//
/* Check whether node is feasible, i.e., all dummy variables are zero. */
void check_feasibility(bool &node_is_feasible, vector<Vehicle> &vehicles, GRBModel &master_problem) {
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        if (master_problem.getVarByName("dummy_" + to_string((*v_it).Id)).get(GRB_DoubleAttr_X) >= 0.00001) {
            node_is_feasible = false;
            break;
        }
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Determine split route assignments in current LP solution and check integrality of vehicles' routes. */
void find_split_routes(vector<VecD> &node_split_routes, bool &node_is_integral, vector<Vehicle> &vehicles, GRBModel &master_problem) {
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        VecD split_values;
        for (size_t r = 0; r != (*v_it).All_routes.size(); ++r) {
            if (find((*v_it).Available_routes.begin(), (*v_it).Available_routes.end(), r) == (*v_it).Available_routes.end()) {
                split_values.push_back(0.0);
            }
            else {
                double var_value(master_problem.getVarByName("use_route_" + to_string((*v_it).Id) + "_" + to_string(r)).get(GRB_DoubleAttr_X));
                split_values.push_back(var_value);
                if (var_value >= 0.00001 && var_value <= 0.99999) {
                    node_is_integral = false;
                }
            }
        }
        node_split_routes.push_back(split_values);
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Determine split location usages in current LP solution. */
void find_split_usage(vector<vector<VecD>> &node_split_usage, vector<Vehicle> &vehicles, vector<Location> &locations,
        GRBModel &master_problem, int &number_locations, int &horizon) {
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        VecD dummy(horizon, 0.0); // initialize split usage
        vector<VecD> split_usage_v(number_locations, dummy);
        node_split_usage.push_back(split_usage_v);
        // set values
        for (size_t r_index = 0; r_index != (*v_it).Available_routes.size(); ++r_index) {
            int r((*v_it).Available_routes[r_index]);
            double var_value(master_problem.getVarByName("use_route_" + to_string((*v_it).Id) + "_" + to_string(r)).get(GRB_DoubleAttr_X));
            if (var_value > 0) {
                for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
                    if ((*l_it).Id != 0 && (*l_it).Id != number_locations - 1) {
                        for (int z = (*v_it).Route_start_times[r][(*l_it).Id]; z != (*v_it).Route_end_times[r][(*l_it).Id]; ++z) {
                            node_split_usage[(*v_it).Id][(*l_it).Id][z] += var_value;
                        }
                    }
                }
            }
        }
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Determine split task-location assignments and split task start times in current LP solution. */
void find_split_tasks_and_times(vector<vector<VecD>> &node_split_tasks, vector<vector<VecD>> &node_split_times,
        vector<Model> &models, vector<Vehicle> &vehicles, GRBModel &master_problem,
        int &number_locations, int &number_tasks, int &horizon) {
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        Model model(models[(*v_it).Type]);
        VecD dummy1(number_tasks, 0.0); // initialize split tasks
        vector<VecD> split_tasks_v(number_locations, dummy1);
        node_split_tasks.push_back(split_tasks_v);
        VecD dummy2(horizon, 0.0); // initialize split times
        vector<VecD> split_times_v(number_tasks, dummy2);
        node_split_times.push_back(split_times_v);
        // set values
        for (size_t r_index = 0; r_index != (*v_it).Available_routes.size(); ++r_index) {
            int r((*v_it).Available_routes[r_index]);
            double var_value(master_problem.getVarByName("use_route_" + to_string((*v_it).Id) + "_" + to_string(r)).get(GRB_DoubleAttr_X));
            if (var_value > 0) {
                for (auto x_it = (*v_it).Route_details[r].begin(); x_it != (*v_it).Route_details[r].end(); ++x_it) {
                    for (auto t = model.Possible_task_location_combinations_list[(*x_it)[0]][(*x_it)[1]].begin(); t != model.Possible_task_location_combinations_list[(*x_it)[0]][(*x_it)[1]].end(); ++t) {
                        if (*t != 0 && *t != number_tasks - 1) {
                            node_split_tasks[(*v_it).Id][(*x_it)[0]][*t] += var_value;
                            node_split_times[(*v_it).Id][*t][(*x_it)[2]] += var_value;
                        }
                    }
                }
            }
        }
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Determine scts of vehicles (linear combination) in current LP solution. */
void derive_vehicle_scts(vector<Vehicle> &vehicles, GRBModel &master_problem) {
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        double sct(0.0);
        for (size_t r_index = 0; r_index != (*v_it).Available_routes.size(); ++r_index) {
            sct += (*v_it).Route_scts[(*v_it).Available_routes[r_index]] * master_problem.getVarByName("use_route_" + to_string((*v_it).Id) + "_" + to_string((*v_it).Available_routes[r_index])).get(GRB_DoubleAttr_X);
        }
        sct = round(sct * 10000.0) / 10000.0;
        (*v_it).Current_sct = sct;
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Check if node LP solution is integral in routes (without timings). */
void check_integrality_of_routes_without_timings(bool &node_is_integral_without_timings, VecI &used_routes_without_timings, vector<Vehicle> &vehicles, vector<VecD> &node_split_routes) {
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        vector<VecI> used_route_without_timings_v;
        bool initial_route_set(false);
        for (size_t r = 0; r != (*v_it).All_routes.size(); ++r) {
            if (node_split_routes[(*v_it).Id][r] >= 0.00001) {
                if (!initial_route_set) {
                    for (auto x_it = (*v_it).Route_details[r].begin(); x_it != (*v_it).Route_details[r].end(); ++x_it) {
                        used_route_without_timings_v.push_back({(*x_it)[0], (*x_it)[1]});
                    }
                    used_routes_without_timings.push_back((int) r);
                    initial_route_set = true;
                }
                else {
                    size_t pos = 0;
                    for (auto x_it = (*v_it).Route_details[r].begin(); x_it != (*v_it).Route_details[r].end(); ++x_it) {
                        if ((*x_it)[0] != used_route_without_timings_v[pos][0] || (*x_it)[1] != used_route_without_timings_v[pos][1]) {
                            node_is_integral_without_timings = false;
                            break;
                        }
                        ++pos;
                    }
                    if (!node_is_integral_without_timings)
                        break;
                }
            }
        }
    }
}
//------------------------------------------------------------------------------------------------------------//