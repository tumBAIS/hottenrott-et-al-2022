//------------------------------------------------------------------------------------------------------------//
/* Determine better upper bound. Fix the integral routes without timings in current solution.
 * We encode problem in time-continuous MIP and solve problem using GUROBI solver.*/
//------------------------------------------------------------------------------------------------------------//
using namespace std;
//------------------------------------------------------------------------------------------------------------//
// Type definitions
typedef vector<int> VecI;
typedef vector<bool> VecB;
typedef vector<double> VecD;
typedef vector<string> VecS;
//------------------------------------------------------------------------------------------------------------//
/* Solve timecontinuous MIP UB search using GUROBI. */
void solve_timecontinuous_mip_ub(int &ub, VecI &best_sol, vector<Model> &models, vector<Vehicle> &vehicles, vector<Location> &locations,
                  vector<VecI> &transportation_times, int &number_locations, VecI &used_routes_without_timings,
                  vector<vector<vector<VecI>>> &blockings, double &time_limit, clock_t &start_scenario, bool &warmstart, bool &nf) {
    int horizon(vehicles.back().Arrival_time + ub);
    //initialize Gurobi problem
    // create Gurobi environment
    GRBEnv env = new GRBEnv();
    env.start();
    // create empty model
    GRBModel problem = GRBModel(env);
    problem.set(GRB_IntParam_OutputFlag, 0);
    problem.set(GRB_DoubleParam_TimeLimit, max(0.0, double(time_limit - double(double(clock() - start_scenario) / CLOCKS_PER_SEC))));
    // define decisions
    vector<vector<GRBVar>> var_start_times;
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        vector<GRBVar> var_start_times_v;
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            string var_name("start_times_" + to_string((*v_it).Id) + "_" + to_string((*l_it).Id));
            var_start_times_v.push_back(problem.addVar(0.0, horizon, 0.0, GRB_CONTINUOUS, GRBColumn(), var_name));
        }
        var_start_times.push_back(var_start_times_v);
    }
    vector<vector<vector<GRBVar>>> var_precedences;
    for (auto v1_it = vehicles.begin(); v1_it != vehicles.end(); ++v1_it) {
        vector<vector<GRBVar>> var_precedences_v1;
        for (auto v2_it = vehicles.begin(); v2_it != vehicles.end(); ++v2_it) {
            vector<GRBVar> var_precedences_v1_v2;
            for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
                string var_name("precedence_" + to_string((*v1_it).Id) + "_" + to_string((*v2_it).Id) + "_" + to_string((*l_it).Id));
                var_precedences_v1_v2.push_back(problem.addVar(0.0, 1.0, 0.0, GRB_BINARY, GRBColumn(), var_name));
            }
            var_precedences_v1.push_back(var_precedences_v1_v2);
        }
        var_precedences.push_back(var_precedences_v1);
    }
    if (warmstart) {
        for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
            for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
                var_start_times[(*v_it).Id][(*l_it).Id].set(GRB_DoubleAttr_Start, (*v_it).Route_start_times[best_sol[(*v_it).Id]][(*l_it).Id]);
            }
        }
    }

    GRBVar var_sct = problem.addVar(0.0, ub, 0.0, GRB_CONTINUOUS, GRBColumn(), "sct");
    // define objective
    problem.setObjective(1.0 * var_sct, GRB_MINIMIZE);
    // define constraints
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        Model model(models[(*v_it).Type]);
        string cstr_name("cstr_arrival_" + to_string((*v_it).Id));
        problem.addConstr(var_start_times[(*v_it).Id][0], GRB_EQUAL, (*v_it).Arrival_time, cstr_name);
        for (size_t x = 0; x != (*v_it).Route_details[used_routes_without_timings[(*v_it).Id]].size() - 1; ++x) {
            int loc1((*v_it).Route_details[used_routes_without_timings[(*v_it).Id]][x][0]);
            int c((*v_it).Route_details[used_routes_without_timings[(*v_it).Id]][x][1]);
            int loc2((*v_it).Route_details[used_routes_without_timings[(*v_it).Id]][x + 1][0]);
            string cstr_sequence_name("cstr_sequence_" + to_string((*v_it).Id) + "_" + to_string(loc1) + "_" + to_string(loc2));
            problem.addConstr(var_start_times[(*v_it).Id][loc2], GRB_GREATER_EQUAL, var_start_times[(*v_it).Id][loc1] + model.Possible_task_location_combination_workloads[loc1][c] + transportation_times[loc1][loc2], cstr_sequence_name);
        }
        string cstr_sct_name("cstr_sct_" + to_string((*v_it).Id));
        problem.addConstr(var_sct, GRB_GREATER_EQUAL, var_start_times[(*v_it).Id][number_locations - 1] - (*v_it).Arrival_time, cstr_sct_name);
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            if ((*l_it).Id == 0 || (*l_it).Id == number_locations - 1)
                continue;
            bool visits1(false);
            int workload1(0);
            for (auto x_it = (*v_it).Route_details[used_routes_without_timings[(*v_it).Id]].begin(); x_it != (*v_it).Route_details[used_routes_without_timings[(*v_it).Id]].end(); ++x_it) {
                if ((*x_it)[0] == (*l_it).Id) {
                    visits1 = true;
                    workload1 = model.Possible_task_location_combination_workloads[(*l_it).Id][(*x_it)[1]];
                    break;
                }
            }
            if (!visits1)
                continue;
            for (auto v2_it = vehicles.begin(); v2_it != vehicles.end(); ++v2_it) {
                if ((*v2_it).Id <= (*v_it).Id)
                    continue;
                Model model2(models[(*v2_it).Type]);
                bool visits2(false);
                int workload2(0);
                for (auto x_it = (*v2_it).Route_details[used_routes_without_timings[(*v2_it).Id]].begin(); x_it != (*v2_it).Route_details[used_routes_without_timings[(*v2_it).Id]].end(); ++x_it) {
                    if ((*x_it)[0] == (*l_it).Id) {
                        visits2 = true;
                        workload2 = model2.Possible_task_location_combination_workloads[(*l_it).Id][(*x_it)[1]];
                        break;
                    }
                }
                if (!visits2)
                    continue;
                string cstr_block1_name("block1_" + to_string((*v_it).Id) + "_" + to_string((*v2_it).Id) + "_" + to_string((*l_it).Id));
                problem.addConstr(var_start_times[(*v2_it).Id][(*l_it).Id], GRB_GREATER_EQUAL, var_start_times[(*v_it).Id][(*l_it).Id] + workload1 - horizon * (1 - var_precedences[(*v_it).Id][(*v2_it).Id][(*l_it).Id]), cstr_block1_name);
                string cstr_block2_name("block2_" + to_string((*v_it).Id) + "_" + to_string((*v2_it).Id) + "_" + to_string((*l_it).Id));
                problem.addConstr(var_start_times[(*v_it).Id][(*l_it).Id], GRB_GREATER_EQUAL, var_start_times[(*v2_it).Id][(*l_it).Id] + workload2 - horizon * var_precedences[(*v_it).Id][(*v2_it).Id][(*l_it).Id], cstr_block2_name);
            }
        }
    }
    if (nf) {
        for (auto v1_it = vehicles.begin(); v1_it != vehicles.end(); ++v1_it) {
            Model model1(models[(*v1_it).Type]);
            for (auto v2_it = vehicles.begin(); v2_it != vehicles.end(); ++v2_it) {
                if ((*v1_it).Type == (*v2_it).Type && (*v1_it).Arrival_time < (*v2_it).Arrival_time) {
                    for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
                        if ((*l_it).Id == 0 || (*l_it).Id == number_locations - 1)
                            continue;
                        string cstr_fix_name("fix_precedence_" + to_string((*v1_it).Id) + "_" + to_string((*v2_it).Id) + "_" + to_string((*l_it).Id));
                        problem.addConstr(var_precedences[(*v1_it).Id][(*v2_it).Id][(*l_it).Id], GRB_EQUAL, 1, cstr_fix_name);
                    }
                }
            }
        }
    }
    problem.set(GRB_DoubleParam_BestBdStop, ub + 0.01);
    problem.update();
    // solve problem: min sct
    problem.optimize();
    if ((problem.get(GRB_IntAttr_Status) == 2 || problem.get(GRB_IntAttr_Status) == 15) && (int) round(problem.get(GRB_DoubleAttr_ObjVal)) < ub) {
        // analyze solution
        int sol = (int) round(var_sct.get(GRB_DoubleAttr_X));
        if (sol < ub) {
            ub = sol;
            cout << "NEW BEST (MIP-TC): " << ub << endl;
            VecI new_best_sol;
            for (int v = 0; v != (int) vehicles.size(); ++v) {
                vector<VecI> new_route_details;
                VecI start_times(number_locations, 0);
                VecI end_times(number_locations, 0);
                int last_end_time(0);
                int last_end_loc(0);
                for (auto x_it = vehicles[v].Route_details[used_routes_without_timings[v]].begin(); x_it != vehicles[v].Route_details[used_routes_without_timings[v]].end(); ++x_it) {
                    int start_time((int) round(problem.getVarByName("start_times_" + to_string(v) + "_" + to_string((*x_it)[0])).get(GRB_DoubleAttr_X)));
                    int end_time(start_time + models[vehicles[v].Type].Possible_task_location_combination_workloads[(*x_it)[0]][(*x_it)[1]]);
                    if ((*x_it)[0] == number_locations - 1) {
                        start_time = last_end_time + transportation_times[last_end_loc][(*x_it)[0]];
                        end_time = start_time;
                    }
                    new_route_details.push_back({(*x_it)[0], (*x_it)[1], start_time});
                    start_times[(*x_it)[0]] = start_time;
                    end_times[(*x_it)[0]] = end_time;
                    last_end_time = end_time;
                    last_end_loc = (*x_it)[0];
                }
                if (find(vehicles[v].Route_details.begin(), vehicles[v].Route_details.end(), new_route_details) == vehicles[v].Route_details.end()) {
                    int index((int) vehicles[v].All_routes.size());
                    vehicles[v].All_routes.push_back(index); // adding routes to be available might be wrong for current branching!!!
                    vehicles[v].Route_details.push_back(new_route_details);
                    vehicles[v].Route_scts.push_back(new_route_details.back()[2] - vehicles[v].Arrival_time);
                    vehicles[v].Route_start_times.push_back(start_times);
                    vehicles[v].Route_end_times.push_back(end_times);
                    for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
                        for (int z = start_times[(*l_it).Id]; z != end_times[(*l_it).Id]; ++z) {
                            blockings[(*l_it).Id][z].push_back({v, index});
                        }
                    }
                    new_best_sol.push_back(index);
                }
                else {
                    auto it = find(vehicles[v].Route_details.begin(), vehicles[v].Route_details.end(), new_route_details);
                    new_best_sol.push_back((int) distance(vehicles[v].Route_details.begin(), it));
                }
            }
            best_sol = new_best_sol;
        }
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Solve timecontinuous MIP OF search using GUROBI. */
void solve_timecontinuous_mip_of(int &ub, VecI &best_sol, vector<Model> &models, vector<Vehicle> &vehicles, vector<Location> &locations,
                                 vector<VecI> &transportation_times, int &number_locations, VecI &used_routes_without_timings,
                                 vector<vector<vector<VecI>>> &blockings, double &time_limit, clock_t &start_scenario, bool &warmstart) {
    vector<vector<VecI>> arcs;
    vector<VecI> used_locations;
    vector<VecI> workloads;
    vector<vector<VecI>> precedence_relations;
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        vector<VecI> arcs_v;
        VecI used_locations_v;
        vector<VecI> precedence_relations_v;
        for (auto x_it = (*v_it).Route_details[used_routes_without_timings[(*v_it).Id]].begin(); x_it != (*v_it).Route_details[used_routes_without_timings[(*v_it).Id]].end(); ++x_it) {
            used_locations_v.push_back((*x_it)[0]);
        }
        used_locations.push_back(used_locations_v);
        for (auto l1 = used_locations_v.begin(); l1 != used_locations_v.end(); ++l1) {
            for (auto l2 = used_locations_v.begin(); l2 != used_locations_v.end(); ++l2) {
                if (*l1 == *l2 || locations[*l1].Level_index > locations[*l2].Level_index)
                    continue;
                if (*l2 == 0 || *l1 == number_locations - 1)
                    continue;
                bool feasible(true);
                for (auto l3 = used_locations_v.begin(); l3 != used_locations_v.end(); ++l3) {
                    if (*l3 == *l1 || *l3 == *l2)
                        continue;
                    if (locations[*l3].Level_index > locations[*l1].Level_index && locations[*l3].Level_index < locations[*l2].Level_index) {
                        feasible = false;
                        break;
                    }
                    if (*l1 == 0 && locations[*l3].Level_index < locations[*l2].Level_index) {
                        feasible = false;
                        break;
                    }
                    if (*l2 == number_locations - 1 && locations[*l1].Level_index < locations[*l3].Level_index) {
                        feasible = false;
                        break;
                    }
                }
                int c1(0), c2(0);
                for (auto x_it = (*v_it).Route_details[used_routes_without_timings[(*v_it).Id]].begin(); x_it != (*v_it).Route_details[used_routes_without_timings[(*v_it).Id]].end(); ++x_it) {
                    if ((*x_it)[0] == *l1) {
                        c1 = (*x_it)[1];
                    }
                    if ((*x_it)[0] == *l2) {
                        c2 = (*x_it)[1];
                    }
                }
                if (locations[*l1].Level_index == locations[*l2].Level_index) {
                    for (auto t1 = models[(*v_it).Type].Possible_task_location_combinations_list[*l1][c1].begin(); t1 != models[(*v_it).Type].Possible_task_location_combinations_list[*l1][c1].end(); ++t1) {
                        for (auto t2 = models[(*v_it).Type].Possible_task_location_combinations_list[*l2][c2].begin(); t2 != models[(*v_it).Type].Possible_task_location_combinations_list[*l2][c2].end(); ++t2) {
                            if (models[(*v_it).Type].Scenario_all_predecessors[*t1][*t2]) {
                                feasible = false;
                                VecI element_to_be_found({*l2, *l1});
                                if (find(precedence_relations_v.begin(), precedence_relations_v.end(), element_to_be_found) == precedence_relations_v.end()) {
                                    precedence_relations_v.push_back(element_to_be_found);
                                }
                            }
                        }
                    }
                }
                if (feasible) {
                    arcs_v.push_back({*l1, *l2});
                }
            }
        }
        arcs.push_back(arcs_v);
        precedence_relations.push_back(precedence_relations_v);
    }
    int horizon(vehicles.back().Arrival_time + ub);
    //initialize Gurobi problem
    // create Gurobi environment
    GRBEnv env = new GRBEnv();
    env.start();
    // create empty model
    GRBModel problem = GRBModel(env);
    problem.set(GRB_IntParam_OutputFlag, 0);
    problem.set(GRB_DoubleParam_TimeLimit, max(0.0, double(time_limit - double(double(clock() - start_scenario) / CLOCKS_PER_SEC))));
    // define decisions
    vector<vector<GRBVar>> var_use_arc;
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        vector<GRBVar> var_use_arc_v;
        for (auto a = arcs[(*v_it).Id].begin(); a != arcs[(*v_it).Id].end(); ++a) {
            string var_name("use_arc_" + to_string((*v_it).Id) + "_" + to_string((*a)[0]) + "_" + to_string((*a)[1]));
            var_use_arc_v.push_back(problem.addVar(0.0, 1.0, 0.0, GRB_BINARY, GRBColumn(), var_name));
        }
        var_use_arc.push_back(var_use_arc_v);
    }
    vector<vector<GRBVar>> var_start_times;
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        vector<GRBVar> var_start_times_v;
        for (auto l_it = used_locations[(*v_it).Id].begin(); l_it != used_locations[(*v_it).Id].end(); ++l_it) {
            string var_name("start_times_" + to_string((*v_it).Id) + "_" + to_string(*l_it));
            var_start_times_v.push_back(problem.addVar(0.0, horizon, 0.0, GRB_CONTINUOUS, GRBColumn(), var_name));
        }
        var_start_times.push_back(var_start_times_v);
    }
    vector<vector<vector<GRBVar>>> var_precedences;
    for (auto v1_it = vehicles.begin(); v1_it != vehicles.end(); ++v1_it) {
        vector<vector<GRBVar>> var_precedences_v1;
        for (auto v2_it = vehicles.begin(); v2_it != vehicles.end(); ++v2_it) {
            vector<GRBVar> var_precedences_v1_v2;
            for (auto l_it = used_locations[(*v1_it).Id].begin(); l_it != used_locations[(*v1_it).Id].end(); ++l_it) {
                if (find(used_locations[(*v2_it).Id].begin(), used_locations[(*v2_it).Id].end(), (*l_it)) == used_locations[(*v2_it).Id].end())
                    continue;
                string var_name("precedence_" + to_string((*v1_it).Id) + "_" + to_string((*v2_it).Id) + "_" + to_string(*l_it));
                var_precedences_v1_v2.push_back(problem.addVar(0.0, 1.0, 0.0, GRB_BINARY, GRBColumn(), var_name));
            }
            var_precedences_v1.push_back(var_precedences_v1_v2);
        }
        var_precedences.push_back(var_precedences_v1);
    }
    if (warmstart) {
        for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
            int l_count(0);
            for (auto l_it = used_locations[(*v_it).Id].begin(); l_it != used_locations[(*v_it).Id].end(); ++l_it, ++l_count) {
                var_start_times[(*v_it).Id][l_count].set(GRB_DoubleAttr_Start, (*v_it).Route_start_times[best_sol[(*v_it).Id]][*l_it]);
            }
        }
    }
    GRBVar var_sct = problem.addVar(0.0, ub, 0.0, GRB_CONTINUOUS, GRBColumn(), "sct");
    problem.update();
    // define objective
    problem.setObjective(1.0 * var_sct, GRB_MINIMIZE);
    // define constraints
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        Model model(models[(*v_it).Type]);
        string cstr_name("cstr_arrival_" + to_string((*v_it).Id));
        problem.addConstr(var_start_times[(*v_it).Id][0], GRB_EQUAL, (*v_it).Arrival_time, cstr_name);
        for (auto l_it = used_locations[(*v_it).Id].begin(); l_it != used_locations[(*v_it).Id].end(); ++l_it) {
            GRBLinExpr arcs_out = 0;
            GRBLinExpr arcs_in = 0;
            int count(0);
            for (auto a = arcs[(*v_it).Id].begin(); a != arcs[(*v_it).Id].end(); ++a, ++count) {
                if ((*a)[0] == *l_it) {
                    arcs_out += var_use_arc[(*v_it).Id][count];
                }
                if ((*a)[1] == *l_it) {
                    arcs_in += var_use_arc[(*v_it).Id][count];
                }
            }
            string cstr1_name("flow_conservation_out_" + to_string((*v_it).Id) + "_" + to_string(*l_it));
            string cstr2_name("flow_conservation_in_" + to_string((*v_it).Id) + "_" + to_string(*l_it));
            if (*l_it == 0) {
                problem.addConstr(arcs_out, GRB_EQUAL, 1, cstr1_name);
            }
            else if (*l_it == number_locations - 1) {
                problem.addConstr(arcs_in, GRB_EQUAL, 1, cstr2_name);
            }
            else {
                problem.addConstr(arcs_out, GRB_EQUAL, 1, cstr1_name);
                problem.addConstr(arcs_in, GRB_EQUAL, 1, cstr2_name);
            }
        }
        int count(0);
        for (auto a = arcs[(*v_it).Id].begin(); a != arcs[(*v_it).Id].end(); ++a, ++count) {
            string cstr_name("time_propagation_" + to_string((*v_it).Id) + "_" + to_string((*a)[0]) + "_" + to_string((*a)[1]));
            problem.addConstr(problem.getVarByName("start_times_" + to_string((*v_it).Id) + "_" + to_string((*a)[1])), GRB_GREATER_EQUAL, problem.getVarByName("start_times_" + to_string((*v_it).Id) + "_" + to_string((*a)[0])) + model.Preselect_route_workloads[(*a)[0]] + transportation_times[(*a)[0]][(*a)[1]] - horizon * (1 - var_use_arc[(*v_it).Id][count]), cstr_name);
        }
        string cstr_sct_name("cstr_sct_" + to_string((*v_it).Id));
        problem.addConstr(var_sct, GRB_GREATER_EQUAL, problem.getVarByName("start_times_" + to_string((*v_it).Id) + "_" + to_string(number_locations - 1)) - (*v_it).Arrival_time, cstr_sct_name);
        // precedence relations among vehicles
        for (auto l_it = used_locations[(*v_it).Id].begin(); l_it != used_locations[(*v_it).Id].end(); ++l_it) {
            if (*l_it == 0 || *l_it == number_locations - 1)
                continue;
            for (auto v2_it = vehicles.begin(); v2_it != vehicles.end(); ++v2_it) {
                if ((*v2_it).Id <= (*v_it).Id)
                    continue;
                Model model2(models[(*v2_it).Type]);
                if (find(used_locations[(*v2_it).Id].begin(), used_locations[(*v2_it).Id].end(), *l_it) == used_locations[(*v2_it).Id].end())
                    continue;
                string cstr_block1_name("block1_" + to_string((*v_it).Id) + "_" + to_string((*v2_it).Id) + "_" + to_string(*l_it));
                problem.addConstr(problem.getVarByName("start_times_" + to_string((*v2_it).Id) + "_" + to_string(*l_it)), GRB_GREATER_EQUAL, problem.getVarByName("start_times_" + to_string((*v_it).Id) + "_" + to_string(*l_it)) + model.Preselect_route_workloads[*l_it] - horizon * (1 - problem.getVarByName("precedence_" + to_string((*v_it).Id) + "_" + to_string((*v2_it).Id) + "_" + to_string(*l_it))), cstr_block1_name);
                string cstr_block2_name("block2_" + to_string((*v_it).Id) + "_" + to_string((*v2_it).Id) + "_" + to_string(*l_it));
                problem.addConstr(problem.getVarByName("start_times_" + to_string((*v_it).Id) + "_" + to_string(*l_it)), GRB_GREATER_EQUAL, problem.getVarByName("start_times_" + to_string((*v2_it).Id) + "_" + to_string(*l_it)) + model2.Preselect_route_workloads[*l_it] - horizon * problem.getVarByName("precedence_" + to_string((*v_it).Id) + "_" + to_string((*v2_it).Id) + "_" + to_string(*l_it)), cstr_block2_name);
            }
        }
        // precedence relations among locations on same level
        for (auto p = precedence_relations[(*v_it).Id].begin(); p != precedence_relations[(*v_it).Id].end(); ++p) {
            string cstr_name("precedence_" + to_string((*v_it).Id) + "_" + to_string((*p)[0]) + "_" + to_string((*p)[1]));
            problem.addConstr(problem.getVarByName("start_times_" + to_string((*v_it).Id) + "_" + to_string((*p)[1])), GRB_GREATER_EQUAL,  problem.getVarByName("start_times_" + to_string((*v_it).Id) + "_" + to_string((*p)[0])) + model.Preselect_route_workloads[(*p)[0]], cstr_name);
        }
    }

    problem.set(GRB_DoubleParam_BestBdStop, ub - 0.99);
    problem.update();
    // solve problem: min sct
    problem.optimize();

    if ((problem.get(GRB_IntAttr_Status) == 2 || problem.get(GRB_IntAttr_Status) == 15) && (int) round(problem.get(GRB_DoubleAttr_ObjVal)) < ub) {
        // analyze solution
        int sol = (int) round(var_sct.get(GRB_DoubleAttr_X));
        if (sol < ub) {
            ub = sol;
            cout << "NEW BEST (MIP-TC): " << ub << endl;
            VecI new_best_sol;
            for (int v = 0; v != (int) vehicles.size(); ++v) {
                vector<VecI> new_route_details;
                VecI start_times(number_locations, 0);
                VecI end_times(number_locations, 0);
                int last_end_time(0);
                int last_end_loc(0);
                new_route_details.push_back({0, 0, vehicles[v].Arrival_time});
                while (last_end_loc != number_locations - 1) {
                    int end_loc(last_end_loc);
                    int count(0);
                    for (auto a = arcs[v].begin(); a != arcs[v].end(); ++a, ++count) {
                        if ((*a)[0] == last_end_loc && var_use_arc[v][count].get(GRB_DoubleAttr_X) > 0.9999) {
                            end_loc = (*a)[1];
                            break;
                        }
                    }
                    int start_time((int) round(problem.getVarByName("start_times_" + to_string(v) + "_" + to_string(end_loc)).get(GRB_DoubleAttr_X)));
                    int end_time(start_time + models[vehicles[v].Type].Preselect_route_workloads[end_loc]);
                    if (end_loc == number_locations - 1) {
                        start_time = last_end_time + transportation_times[last_end_loc][end_loc];
                        end_time = start_time;
                    }
                    int c(0);
                    for (auto x_it = models[vehicles[v].Type].Preselect_route.begin(); x_it != models[vehicles[v].Type].Preselect_route.end(); ++x_it) {
                        if ((*x_it)[0] == end_loc) {
                            c = (*x_it)[1];
                            break;
                        }
                    }
                    new_route_details.push_back({end_loc, c, start_time});
                    start_times[end_loc] = start_time;
                    end_times[end_loc] = end_time;
                    last_end_time = end_time;
                    last_end_loc = end_loc;

                }
                if (find(vehicles[v].Route_details.begin(), vehicles[v].Route_details.end(), new_route_details) == vehicles[v].Route_details.end()) {
                    int index((int) vehicles[v].All_routes.size());
                    vehicles[v].All_routes.push_back(index); // adding routes to be available might be wrong for current branching!!!
                    vehicles[v].Route_details.push_back(new_route_details);
                    vehicles[v].Route_scts.push_back(new_route_details.back()[2] - vehicles[v].Arrival_time);
                    vehicles[v].Route_start_times.push_back(start_times);
                    vehicles[v].Route_end_times.push_back(end_times);
                    for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
                        for (int z = start_times[(*l_it).Id]; z != end_times[(*l_it).Id]; ++z) {
                            blockings[(*l_it).Id][z].push_back({v, index});
                        }
                    }
                    new_best_sol.push_back(index);
                }
                else {
                    auto it = find(vehicles[v].Route_details.begin(), vehicles[v].Route_details.end(), new_route_details);
                    new_best_sol.push_back((int) distance(vehicles[v].Route_details.begin(), it));
                }
            }
            best_sol = new_best_sol;
        }
    }
}
//------------------------------------------------------------------------------------------------------------//
