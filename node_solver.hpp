//------------------------------------------------------------------------------------------------------------//
/* Solve LP relaxation of node in B&B tree using column generation. */
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
/* Solve node.*/
void solve_node(int &horizon, int &ub, int &ub_escape, VecI &best_sol, double &node_lb, bool &node_is_integral, bool &node_is_feasible, vector<VecD> &node_split_routes,
        vector<vector<VecD>> &node_split_usage, vector<vector<VecD>> &node_split_tasks, vector<vector<VecD>> &node_split_times,
        Node parent_node, vector<Model> &models, vector<Vehicle> &vehicles, vector<Location> &locations, vector<vector<vector<VecI>>> &blockings,
        vector<VecI> &transportation_times, int &number_tasks, int &number_locations, bool &master_activity_end, int &max_routes_per_vehicle,
        double &time_limit, clock_t &start_scenario, double &time_create_mp, double &time_solve_mp, double &time_identify_cuts, double &time_read_duals, double &time_solve_sp, double &time_add_columns,
        double &time_mip_search, double &time_sp_identify_extension, double &time_sp_perform_extension, double &time_sp_dominance_check_1, double &time_sp_dominance_check_2,
        double &time_add_fragment, double &time_preprocess_node, double &time_postprocess_node, double &time_check_integrality_and_update_best) {
    // prepare node
    clock_t start_preprocess_node = clock();
    vector<vector<vector<VecI>>> available_blockings;
    preprocess_node(models, vehicles, locations, blockings, available_blockings, parent_node, number_tasks, number_locations, ub, horizon);
    time_preprocess_node += double(clock() - start_preprocess_node) / CLOCKS_PER_SEC;

    // build Master LP
    clock_t start_create_mp = clock();
    // create Gurobi environment
    GRBEnv env = new GRBEnv();
    env.start();
    // create empty model
    GRBModel master_problem = GRBModel(env);
    GRBVar segment_cycle_time;
    vector<vector<GRBVar>> use_route;
    vector<GRBVar> dummy;
    vector<VecI> active_ends;
    vector<vector<VecI>> zero_half_cuts;
    create_master(master_activity_end, master_problem, segment_cycle_time, use_route, dummy, active_ends, vehicles, locations, number_locations, available_blockings, horizon);
    time_create_mp += double(clock() - start_create_mp) / CLOCKS_PER_SEC;
    int cg_iteration_count(1);
    while (1) { // start column generation iterations
        // solve master problem
        clock_t start_solve_mp = clock();
        master_problem.set(GRB_DoubleParam_TimeLimit, max(0.0, double(time_limit - double(double(clock() - start_scenario) / CLOCKS_PER_SEC))));
        master_problem.optimize();
        time_solve_mp += double(clock() - start_solve_mp) / CLOCKS_PER_SEC;
        if (master_problem.get(GRB_IntAttr_Status) != 2) {
            cout << "WARNING: Master problem is infeasible!" << endl;
            return;
        }
        node_lb = master_problem.get(GRB_DoubleAttr_ObjVal);
        // check if master LP solution is integral
        clock_t start_check_integrality_and_update_best = clock();
        check_integrality(node_is_integral, vehicles, use_route);
        node_is_feasible = true;
        check_feasibility(node_is_feasible, vehicles, master_problem);
        // check if new upper bound is found
        if (node_is_feasible && node_is_integral && (int) round(node_lb) < ub) {
            update_best(ub, node_lb, segment_cycle_time, best_sol, vehicles, use_route);
        }
        time_check_integrality_and_update_best += double(clock() - start_check_integrality_and_update_best) / CLOCKS_PER_SEC;
        if (ub <= ub_escape) {
            break;
        }
        if (node_lb <= parent_node.Lb) {
            break;
        }
        // derive master problem duals
        clock_t start_read_duals = clock();
        vector<VecD> block_cost;
        VecD cut_cost;
        derive_duals(master_problem, vehicles, locations, block_cost, cut_cost, number_locations, horizon, available_blockings, zero_half_cuts);
        time_read_duals += double(clock() - start_read_duals) / CLOCKS_PER_SEC;
        // solve subproblems
        clock_t start_solve_sp = clock();
        bool promising_route_found(false);
        vector<vector<VecI>> dummy;
        vector<vector<vector<VecI>>> new_routes(vehicles.size(), dummy);
        VecD dummy2({});
        vector<VecD> new_routes_cost(vehicles.size(), dummy2);
        vector<Vehicle> shuffled_vehicles(vehicles);
        random_shuffle(shuffled_vehicles.begin(), shuffled_vehicles.end());
        int success(0);
        for (auto v_it = shuffled_vehicles.begin(); v_it != shuffled_vehicles.end(); ++v_it) {
            if (success >= 5)
                break;
            vector<vector<VecI>> new_routes_v;
            VecD new_routes_cost_v;
            if ((*v_it).Max_cost > models[(*v_it).Type].Min_sct * (*v_it).Sct_cost) {
                find_new_routes(new_routes_v, new_routes_cost_v, (*v_it).Id, (*v_it), models[(*v_it).Type], locations, block_cost, cut_cost, zero_half_cuts, transportation_times, number_locations, number_tasks, parent_node, ub, horizon,
                                time_sp_identify_extension, time_sp_perform_extension, time_sp_dominance_check_1, time_sp_dominance_check_2, time_add_fragment);
                if (new_routes_v.size() > 0) {
                    promising_route_found = true;
                    ++success;
                }
                if ((int) new_routes_v.size() > max_routes_per_vehicle) {
                    struct route {
                        int Id;
                        vector<VecI> Route;
                        double Cost;
                    };
                    vector<route> route_list_v;
                    for (int x = 0; x != new_routes_v.size(); ++x) {
                        route new_route;
                        new_route.Id = x;
                        new_route.Route = new_routes_v[x];
                        new_route.Cost = new_routes_cost_v[x];
                        route_list_v.push_back(new_route);
                    }
                    sort(route_list_v.begin(), route_list_v.end(), [](const auto &r1, const auto &r2) {return r1.Id < r2.Id;});
                    sort(route_list_v.begin(), route_list_v.end(), [](const auto &r1, const auto &r2) {return r1.Cost < r2.Cost;});
                    vector<vector<VecI>> new_new_routes_v;
                    VecD new_new_routes_cost_v;
                    int count = 0;
                    for (auto it = route_list_v.begin(); it != route_list_v.end(); ++it, ++count) {
                        if (count == max_routes_per_vehicle) {
                            break;
                        }
                        new_new_routes_v.push_back((*it).Route);
                        new_new_routes_cost_v.push_back((*it).Cost);
                    }
                    new_routes_v = new_new_routes_v;
                    new_routes_cost_v = new_new_routes_cost_v;
                }
            }
            new_routes[(*v_it).Id] = new_routes_v;
            new_routes_cost[(*v_it).Id] = new_routes_cost_v;
        }
        time_solve_sp += double(clock() - start_solve_sp) / CLOCKS_PER_SEC;

        if (!promising_route_found) { // exit loop if no more promising routes
            break;
        }
        // add columns
        clock_t start_add_columns = clock();
        for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
            for (size_t x = 0; x != new_routes[(*v_it).Id].size(); ++x) {
                Model model(models[(*v_it).Type]);
                auto it(find((*v_it).Route_details.begin(), (*v_it).Route_details.end(), new_routes[(*v_it).Id][x]));
                if (it != vehicles[(*v_it).Id].Route_details.end()) {
                    cout << "Create existing route again!" << endl;
                    cout << "V" << (*v_it).Id << endl;
                    VecI task_seq;
                    for (auto x_it = (*it).begin(); x_it != (*it).end(); ++ x_it) {
                        cout << "(" << (*x_it)[0] <<","<<(*x_it)[1] <<","<<(*x_it)[2] << "," <<(*x_it)[2] + model.Possible_task_location_combination_workloads[(*x_it)[0]][(*x_it)[1]]  <<") ";
                        for (auto t_it = model.Possible_task_location_combinations_list[(*x_it)[0]][(*x_it)[1]].begin(); t_it != model.Possible_task_location_combinations_list[(*x_it)[0]][(*x_it)[1]].end(); ++t_it) {
                            task_seq.push_back((*t_it));
                        }
                    }
                    int id((int) distance((*v_it).Route_details.begin(), it));
                    cout << (*v_it).Route_scts[id] << endl;
                    cout << "Task sequence:";
                    for (auto t_it = task_seq.begin(); t_it != task_seq.end(); ++ t_it) {
                        cout << " " << (*t_it);
                    }
                    cout << endl;
                    cout << "Id: " << id << endl;
                    cout << "UB: " << ub << endl;
                    cout << "#Av. routes: " << (*v_it).Available_routes.size() << endl;
                    auto it2(find((*v_it).Available_routes.begin(), (*v_it).Available_routes.end(), id));
                    if (it2 == (*v_it).Available_routes.end())
                        cout << "Route exists but not available" << endl;
                    cout << "Enforced" << endl;
                    for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
                        cout << (*l_it).Id << ":";
                        for (auto it3 = parent_node.Enforced_vehicle_location_times[(*v_it).Id][(*l_it).Id].begin(); it3 != parent_node.Enforced_vehicle_location_times[(*v_it).Id][(*l_it).Id].end(); ++it3) {
                            cout << " " << (*it3);
                        }
                        cout << endl;
                    }
                    cout << "Forbidden" << endl;
                    for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
                        cout << (*l_it).Id << ":";
                        for (auto it3 = parent_node.Forbidden_vehicle_location_times[(*v_it).Id][(*l_it).Id].begin(); it3 != parent_node.Forbidden_vehicle_location_times[(*v_it).Id][(*l_it).Id].end(); ++it3) {
                            cout << " " << (*it3);
                        }
                        cout << endl;
                    }
                    for (auto x_it = new_routes[(*v_it).Id][x].begin(); x_it != new_routes[(*v_it).Id][x].end(); ++ x_it) {
                        cout << "(" << (*x_it)[0] <<","<<(*x_it)[1] <<","<<(*x_it)[2] << "," <<(*x_it)[2] + model.Possible_task_location_combination_workloads[(*x_it)[0]][(*x_it)[1]]  <<") ";
                    }
                    cout << (int) round(new_routes[(*v_it).Id][x].back()[2] - (*v_it).Arrival_time) << endl;
                }
                int index((int) (*v_it).All_routes.size());
                (*v_it).All_routes.push_back(index);
                (*v_it).Available_routes.push_back(index);
                (*v_it).Route_details.push_back(new_routes[(*v_it).Id][x]);
                (*v_it).Route_scts.push_back((int) round(new_routes[(*v_it).Id][x].back()[2] - (*v_it).Arrival_time));
                VecI start_times(number_locations, 0);
                VecI end_times(number_locations, 0);
                for (auto it = new_routes[(*v_it).Id][x].begin(); it != new_routes[(*v_it).Id][x].end(); ++it) {
                    start_times[(*it)[0]] = (int) round((*it)[2]);
                    end_times[(*it)[0]] = (int) round((*it)[2] + model.Possible_task_location_combination_workloads[(*it)[0]][(*it)[1]]);
                }

                (*v_it).Route_start_times.push_back(start_times);
                (*v_it).Route_end_times.push_back(end_times);
                // create new column in master problem
                GRBColumn column = GRBColumn();
                column.addTerm(1, master_problem.getConstrByName("cstr_select_" + to_string((*v_it).Id)));
                column.addTerm(-(new_routes[(*v_it).Id][x].back()[2] - (*v_it).Arrival_time), master_problem.getConstrByName("cstr_sct_" + to_string((*v_it).Id)));
                for (int zhc = 0; zhc != zero_half_cuts.size(); ++zhc) {
                    int collide(0);
                    for (auto c = zero_half_cuts[zhc].begin(); c != zero_half_cuts[zhc].end(); ++c) {
                        if ((*v_it).Route_start_times.back()[(*c)[0]] <= (*c)[1] && (*v_it).Route_end_times.back()[(*c)[0]] > (*c)[1]) {
                            ++collide;
                        }
                    }
                    if (collide >= 2) {
                        column.addTerm(1, master_problem.getConstrByName("cstr_cut_" + to_string(zhc)));
                    }
                }
                if (master_activity_end) {
                    // update existing block constraints with new activity
                    for (auto it = new_routes[(*v_it).Id][x].begin(); it != new_routes[(*v_it).Id][x].end(); ++it) {
                        if ((*it)[0] != 0 && (*it)[0] != number_locations - 1) {
                            for (int z = (*it)[2]; z != (*it)[2] + model.Possible_task_location_combination_workloads[(*it)[0]][(*it)[1]]; ++z) {
                                blockings[(*it)[0]][z].push_back({(*v_it).Id, index});
                                available_blockings[(*it)[0]][z].push_back({(*v_it).Id, index});
                                VecI element_to_be_found({(*it)[0], z});
                                if (find(active_ends.begin(), active_ends.end(), element_to_be_found) != active_ends.end()) {
                                    column.addTerm(1, master_problem.getConstrByName("cstr_block_" + to_string((*it)[0]) + "_" + to_string(z)));
                                }
                            }
                        }
                    }
                    // create new blocking constraints at end points of new route
                    for (auto it = new_routes[(*v_it).Id][x].begin(); it != new_routes[(*v_it).Id][x].end(); ++it) {
                        if ((*it)[0] != 0 && (*it)[0] != number_locations - 1) {
                            VecI element_to_be_found({(*it)[0], (*it)[2] + model.Possible_task_location_combination_workloads[(*it)[0]][(*it)[1]] - 1});
                            if (find(active_ends.begin(), active_ends.end(), element_to_be_found) == active_ends.end()) {
                                column.addTerm(1, master_problem.getConstrByName("cstr_block_" + to_string((*it)[0]) + "_" + to_string((*it)[2] + model.Possible_task_location_combination_workloads[(*it)[0]][(*it)[1]] - 1)));
                                for (size_t i = 0; i != available_blockings[(*it)[0]][(*it)[2] + model.Possible_task_location_combination_workloads[(*it)[0]][(*it)[1]] - 1].size(); ++i) {
                                    int v2(available_blockings[(*it)[0]][(*it)[2] + model.Possible_task_location_combination_workloads[(*it)[0]][(*it)[1]] - 1][i][0]);
                                    int r2(available_blockings[(*it)[0]][(*it)[2] + model.Possible_task_location_combination_workloads[(*it)[0]][(*it)[1]] - 1][i][1]);
                                    if ((*v_it).Id == v2 && index == r2)
                                        continue;
                                    GRBVar var(master_problem.getVarByName("use_route_" + to_string(v2) + "_" + to_string(r2)));
                                    GRBConstr cstr(master_problem.getConstrByName("cstr_block_" + to_string((*it)[0]) + "_" + to_string((*it)[2] + model.Possible_task_location_combination_workloads[(*it)[0]][(*it)[1]] - 1)));
                                    master_problem.chgCoeff(cstr, var, 1);
                                }
                                active_ends.push_back({(*it)[0], (*it)[2] + model.Possible_task_location_combination_workloads[(*it)[0]][(*it)[1]] - 1});
                            }
                        }
                    }
                }
                else {
                    // update block constraints with new activity
                    for (auto it = new_routes[(*v_it).Id][x].begin(); it != new_routes[(*v_it).Id][x].end(); ++it) {
                        if ((*it)[0] != 0 && (*it)[0] != number_locations - 1) {
                            for (int z = (*it)[2]; z != (*it)[2] + model.Possible_task_location_combination_workloads[(*it)[0]][(*it)[1]]; ++z) {
                                blockings[(*it)[0]][z].push_back({(*v_it).Id, index});
                                available_blockings[(*it)[0]][z].push_back({(*v_it).Id, index});
                                column.addTerm(1, master_problem.getConstrByName("cstr_block_" + to_string((*it)[0]) + "_" + to_string(z)));
                            }
                        }
                    }
                }
                string var_name("use_route_" + to_string((*v_it).Id) + "_" + to_string(index));
                use_route[(*v_it).Id].push_back(master_problem.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, column, var_name));
                master_problem.update();
            }
        }
        time_add_columns += double(clock() - start_add_columns) / CLOCKS_PER_SEC;
        ++cg_iteration_count;
    }
    clock_t start_postprocessing_node = clock();
    int total_routes(0);
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        total_routes += (int) (*v_it).Available_routes.size();
    }

    cout << "CG iterations: " << cg_iteration_count  << ", Total routes: " << total_routes << endl;
    // find split routes
    find_split_routes(node_split_routes, node_is_integral, vehicles, master_problem);
    // find split usage
    find_split_usage(node_split_usage, vehicles, locations, master_problem, number_locations, horizon);
    // find split task-location-assignments and split task start times
    find_split_tasks_and_times(node_split_tasks, node_split_times, models, vehicles, master_problem, number_locations, number_tasks, horizon);
    // derive current vehicle scts
    derive_vehicle_scts(vehicles, master_problem);
    time_postprocess_node += double(clock() - start_postprocessing_node) / CLOCKS_PER_SEC;
    horizon = vehicles.back().Arrival_time + ub;
}
//------------------------------------------------------------------------------------------------------------//