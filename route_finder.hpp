//------------------------------------------------------------------------------------------------------------// 
/* Solve subproblem for one vehicle to find a new promising route.
 * We solve a resource-constrained shortest path problem by using a labeling algorithm.*/
//------------------------------------------------------------------------------------------------------------//
using namespace std;
//------------------------------------------------------------------------------------------------------------//
// Type definitions
typedef vector<int> VecI;
typedef vector<bool> VecB;
typedef vector<double> VecD;
typedef vector<string> VecS;
//------------------------------------------------------------------------------------------------------------//
double PRECISION(0.0001);
//------------------------------------------------------------------------------------------------------------//
struct Node {
    int Id; // node identifier
    double Lb; // lower bound
    int Depth; // depth of node in tree
    int Preference; // cumulative branching preference, i.e., how often was branched in the "favorable" direction
    vector<vector<VecI>> Fixed_vehicle_location_tasks; // fixed task locations for the vehicles
    vector<VecI> Task_start_times_lower_bounds; // Lower bounds on the vehicles' task start times
    vector<VecI> Task_start_times_upper_bounds; // Upper bounds on the vehicles' task start times
    vector<vector<VecI>> Enforced_vehicle_location_times; // enforced location times for the vehicles
    vector<vector<VecI>> Forbidden_vehicle_location_times; // forbidden location times for the vehicles
};
//------------------------------------------------------------------------------------------------------------//
/* A route fragment shows a partial route with the corresponding properties. */
struct Route_fragment {
    int Id; // route fragment identifier
    vector<VecI> Route; // sequence of (location, task_combination, time) tuples visited so far
    int Loc; // last location
    int Level_index; // last level index
    int End_time; // last end time
    int Missing_task_time; // cumulated time for tasks that are still missing
    VecB Missing_tasks; // tasks that still need to be performed
    VecB Accessible_locations; // locations that can still be accessed
    double Cost; // route cost so far
    VecI Visits_ZHC; // number of visits of zero-half cut location-time combinations in route fragment
};
//------------------------------------------------------------------------------------------------------------//
/* Extension candidates for route extension of a given parent route fragment.*/
struct Candidate {
    int Id; // ID of extension candidate
    VecI Extension; // tuple of (location, task_combination, start time) that is possible extension
    double Add_cost; // additional cost of route extension candidate
    int End_time; // end time of extension candidate
    int Level_index; // level index of location of extension candidate
    int Row_index; // row index of location of extension candidate
    VecI Visit_ZHC_Change; // new visits of zero-half cut location-time combinations in route extension
};
//------------------------------------------------------------------------------------------------------------//
/* Initialize root route fragment.*/
void initialize_root_route_fragment(Route_fragment &root, Model &model, Vehicle &vehicle, int &number_locations, vector<vector<VecI>> &zero_half_cuts) {
    root.Id = 0;
    root.Route = {{0, 0, vehicle.Arrival_time}};
    root.Loc = 0;
    root.End_time = vehicle.Arrival_time;
    root.Level_index = 1;
    root.Cost = 0.0;
    VecB init_vector(number_locations, true);
    root.Accessible_locations = init_vector;
    root.Accessible_locations[0] = false;
    root.Missing_task_time = model.Workload;
    root.Missing_tasks = model.Tasks_per_model;
    root.Missing_tasks[0] = false;
    VecI init_vector_visits(zero_half_cuts.size(), 0);
    root.Visits_ZHC = init_vector_visits;
}
//------------------------------------------------------------------------------------------------------------//
/* Check whether new route fragment is dominated by existing route fragment. */
bool check_dominance1(vector<Route_fragment> &open_fragments, Route_fragment &new_fragment,
                      int &number_tasks, int &number_locations, double &sct_cost, VecD &cut_cost) {
    bool not_dominated(true);
    for (auto it = open_fragments.begin(); it != open_fragments.end(); ++it) {
        if ((*it).Loc == new_fragment.Loc && (*it).End_time <= new_fragment.End_time) {
            double compare_cost((*it).Cost + sct_cost * (new_fragment.End_time - (*it).End_time));
            for (int c = 0; c != cut_cost.size(); ++c) {
                if ((*it).Visits_ZHC[c] <= 1 && new_fragment.Visits_ZHC[c] >= 2) {
                    compare_cost += cut_cost[c];
                }
            }
            if (compare_cost <= new_fragment.Cost) {
                bool r2_covers_all_tasks_of_new_fragment(true);
                for (size_t t = 0; t != number_tasks; ++t) {
                    if ((*it).Missing_tasks[t] && !new_fragment.Missing_tasks[t]) {
                        r2_covers_all_tasks_of_new_fragment = false;
                        break;
                    }
                }
                if (r2_covers_all_tasks_of_new_fragment) {
                    bool r2_covers_all_accessible_locations_of_new_fragment(true);
                    for (size_t l = 0; l != number_locations; ++l) {
                        if (!(*it).Accessible_locations[l] && new_fragment.Accessible_locations[l]) {
                            r2_covers_all_accessible_locations_of_new_fragment = false;
                            break;
                        }
                    }
                    if (r2_covers_all_accessible_locations_of_new_fragment) {
                        not_dominated = false;
                        break;
                    }
                }
            }
        }
    }
    return not_dominated;
}
//------------------------------------------------------------------------------------------------------------//
/* Check whether new route fragment dominates existing route fragments. */
void check_dominance2(vector<Route_fragment> &open_fragments, Route_fragment &new_fragment,
                      int &number_tasks, int &number_locations, double &sct_cost, VecD &cut_cost) {
    auto it = open_fragments.begin();
    while (it != open_fragments.end()) {
        bool dominated(false);
        if ((*it).Loc == new_fragment.Loc && (*it).End_time >= new_fragment.End_time) {
            double compare_cost(new_fragment.Cost + sct_cost * ((*it).End_time - new_fragment.End_time));
            for (int c = 0; c != cut_cost.size(); ++c) {
                if ((*it).Visits_ZHC[c] >= 2 && new_fragment.Visits_ZHC[c] <= 1) {
                    compare_cost += cut_cost[c];
                }
            }
            if ((*it).Cost >= compare_cost) {
                bool new_fragment_covers_all_tasks_of_r2(true);
                for (size_t t = 0; t != number_tasks; ++t) {
                    if (!(*it).Missing_tasks[t] && new_fragment.Missing_tasks[t]) {
                        new_fragment_covers_all_tasks_of_r2 = false;
                        break;
                    }
                }
                if (new_fragment_covers_all_tasks_of_r2) {
                    bool new_fragment_covers_all_accessible_locations_of_r2(true);
                    for (size_t l = 0; l != number_locations; ++l) {
                        if ((*it).Accessible_locations[l] && !new_fragment.Accessible_locations[l]) {
                            new_fragment_covers_all_accessible_locations_of_r2 = false;
                            break;
                        }
                    }
                    if (new_fragment_covers_all_accessible_locations_of_r2) {
                        dominated = true;
                    }
                }
            }
        }
        if (dominated) {
            open_fragments.erase(it);
        }
        else {
            ++it;
        }
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Find a new promising route for a vehicle. */
void find_new_routes(vector<vector<VecI>> &new_routes, VecD &new_routes_cost, int &v_id, Vehicle &vehicle, Model &model, vector<Location> &locations,
                     vector<VecD> &block_cost, VecD &cut_cost, vector<vector<VecI>> &zero_half_cuts, vector<VecI> &transportation_times, int &number_locations,
                     int &number_tasks, Node &parent_node, int &ub, int &horizon, double &time_sp_identify_extension, double &time_sp_perform_extension,
                     double &time_sp_dominance_check_1, double &time_sp_dominance_check_2, double &time_add_fragment) {
    // initialize root route fragment
    Route_fragment root;
    initialize_root_route_fragment(root, model, vehicle, number_locations, zero_half_cuts);
    // create list of open route fragments
    vector<Route_fragment> open_fragments;
    open_fragments.push_back(root);
    // start route generation
    VecB all_tasks_performed(number_tasks, false);
    int id_count(1);
    while (!open_fragments.empty()) {
        clock_t start_sp_identify_extension = clock();
        // select parent route to be developed further: route that currently ends at location furthest to the left
        Route_fragment parent = open_fragments[0];
        // delete parent route fragment
        open_fragments.erase(open_fragments.begin());
        // identify candidates for extending old route fragment
        vector<Candidate> candidates;
        int cand_id(0);
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            if (!parent.Accessible_locations[(*l_it).Id]) // location needs to be still accessible
                continue;
            int earliest_start(parent.End_time + transportation_times[parent.Loc][(*l_it).Id]);
            // feasibility check: new route needs to finish before end of horizon
            if (earliest_start + parent.Missing_task_time + transportation_times[(*l_it).Id][number_locations - 1] - vehicle.Arrival_time >= ub)
                continue;
            // feasibility check: new route needs to have negative reduced cost
            if (parent.Cost + vehicle.Sct_cost * (transportation_times[parent.Loc][(*l_it).Id] + parent.Missing_task_time + transportation_times[(*l_it).Id][number_locations - 1]) > vehicle.Max_cost - PRECISION)
                continue;
            for (size_t c_index = 0; c_index != vehicle.Available_task_location_combinations[(*l_it).Id].size(); ++c_index) {
                int c(vehicle.Available_task_location_combinations[(*l_it).Id][c_index]);
                bool feasible(true);
                // check if all tasks are still missing
                for (auto t_it = model.Possible_task_location_combinations_list[(*l_it).Id][c].begin(); t_it != model.Possible_task_location_combinations_list[(*l_it).Id][c].end(); ++t_it) {
                    if (!parent.Missing_tasks[*t_it]) {
                        feasible = false;
                        break;
                    }
                }
                if (!feasible)
                    continue;
                // ensure that all precedence relations are met
                for (auto t_it = model.Possible_task_location_combinations_list[(*l_it).Id][c].begin(); t_it != model.Possible_task_location_combinations_list[(*l_it).Id][c].end(); ++t_it) {
                    for (int tp = 0; tp != number_tasks; ++tp) {
                        if (model.Scenario_all_predecessors[*t_it][tp]) {
                            if (parent.Missing_tasks[tp] && !model.Possible_task_location_combinations[(*l_it).Id][c][tp]) {
                                feasible = false;
                                break;
                            }
                        }
                        if (!feasible)
                            break;
                    }
                }
                if (!feasible)
                    continue;
                // feasibility check: transportation arc cannot exist if enforced location at same time
                for (auto l2_it = locations.begin(); l2_it != locations.end(); ++l2_it) {
                    for (auto it = parent_node.Enforced_vehicle_location_times[v_id][(*l2_it).Id].begin(); it != parent_node.Enforced_vehicle_location_times[v_id][(*l2_it).Id].end(); ++it) {
                        if (*it >= parent.End_time && *it < earliest_start) {
                            feasible = false;
                            break;
                        }
                    }
                    if (!feasible)
                        break;
                }
                if (!feasible)
                    continue;
                if ((*l_it).Id == number_locations - 1) {
                    for (auto l2_it = locations.begin(); l2_it != locations.end(); ++l2_it) { //ensure that there are no enforced location times after route end
                        for (auto it = parent_node.Enforced_vehicle_location_times[v_id][(*l2_it).Id].begin(); it != parent_node.Enforced_vehicle_location_times[v_id][(*l2_it).Id].end(); ++it) {
                            if (*it >= earliest_start) {
                                feasible = false;
                                break;
                            }
                        }
                    }
                    if (!feasible)
                        continue;
                    Candidate extension_candidate;
                    extension_candidate.Id = cand_id;
                    extension_candidate.Extension = {(*l_it).Id, c, earliest_start};
                    extension_candidate.Add_cost = vehicle.Sct_cost * (earliest_start - parent.End_time);
                    extension_candidate.End_time = earliest_start;
                    extension_candidate.Level_index = (*l_it).Level_index;
                    candidates.push_back(extension_candidate);
                    ++cand_id;
                }
                else {
                    // derive task sequence
                    int waiting(0);
                    double best_cum_block_cost(vehicle.Max_cost);
                    while (1) {
                        int start_time(earliest_start + waiting);
                        // feasibility check: new route needs to finish before end of horizon
                        if (start_time + parent.Missing_task_time + transportation_times[(*l_it).Id][number_locations - 1] - vehicle.Arrival_time >= ub)
                            break;
                        // feasibility check: new route needs to have negative reduced cost
                        if (parent.Cost + vehicle.Sct_cost * (start_time + parent.Missing_task_time + transportation_times[(*l_it).Id][number_locations - 1] - parent.End_time) > vehicle.Max_cost - PRECISION)
                            break;
                        bool feasible_enforced(true);
                        // feasibility check: arc cannot exist if enforced other location at same time
                        for (auto l2_it = locations.begin(); l2_it != locations.end(); ++l2_it) {
                            if ((*l2_it).Id == (*l_it).Id)
                                continue;
                            for (auto it = parent_node.Enforced_vehicle_location_times[v_id][(*l2_it).Id].begin(); it != parent_node.Enforced_vehicle_location_times[v_id][(*l2_it).Id].end(); ++it) {
                                if ((*it) >= earliest_start && (*it) < start_time + model.Possible_task_location_combination_workloads[(*l_it).Id][c]) {
                                    feasible_enforced = false;
                                    break;
                                }
                            }
                            if (!feasible_enforced)
                                break;
                        }
                        if (!feasible_enforced)
                            break;
                        // check task start time feasibility with branching
                        bool feasible_start_times(true);
                        for (auto t_it = model.Possible_task_location_combinations_list[(*l_it).Id][c].begin(); t_it != model.Possible_task_location_combinations_list[(*l_it).Id][c].end(); ++t_it) {
                            if (parent_node.Task_start_times_lower_bounds[v_id][*t_it] > start_time || parent_node.Task_start_times_upper_bounds[v_id][(*t_it)] < start_time) {
                                feasible_start_times = false;
                                break;
                            }
                        }
                        if (feasible_start_times) {
                            int end_time(start_time + model.Possible_task_location_combination_workloads[(*l_it).Id][c]);
                            bool feasible_location_times(true);
                            for (auto it = parent_node.Enforced_vehicle_location_times[v_id][(*l_it).Id].begin(); it != parent_node.Enforced_vehicle_location_times[v_id][(*l_it).Id].end(); ++it) {
                                if (start_time > *it || end_time - 1 < *it) {
                                    feasible_location_times = false;
                                    break;
                                }
                            }
                            for (auto it = parent_node.Forbidden_vehicle_location_times[v_id][(*l_it).Id].begin(); it != parent_node.Forbidden_vehicle_location_times[v_id][(*l_it).Id].end(); ++it) {
                                if (start_time <= *it && end_time > *it) {
                                    feasible_location_times = false;
                                    break;
                                }
                            }
                            if (feasible_location_times) {
                                double cum_block_cost(0.0);
                                for (int z = start_time; z != end_time; ++z) {
                                    if (block_cost[(*l_it).Id][z] > 0) {
                                        cum_block_cost += block_cost[(*l_it).Id][z];
                                    }
                                }
                                double cum_cut_cost(0.0);
                                VecI visit_extend(zero_half_cuts.size(), 0);
                                for (int zhc = 0; zhc != zero_half_cuts.size(); ++zhc) {
                                    for (auto c = zero_half_cuts[zhc].begin(); c != zero_half_cuts[zhc].end(); ++c) {
                                        if ((*c)[0] == (*l_it).Id && start_time <= (*c)[1] && end_time > (*c)[1]) {
                                            ++visit_extend[zhc];
                                        }
                                    }
                                    if (parent.Visits_ZHC[zhc] == 1 && visit_extend[zhc] >= 1) {
                                        cum_cut_cost += cut_cost[zhc];
                                    }
                                    else if (parent.Visits_ZHC[zhc] == 0 && visit_extend[zhc] >= 2) {
                                        cum_cut_cost += cut_cost[zhc];
                                    }
                                }
                                double additional_cost(vehicle.Sct_cost * (end_time - parent.End_time) + cum_block_cost + cum_cut_cost);
                                // feasibility check: new route needs to have negative reduced cost
                                if (parent.Cost + additional_cost + vehicle.Sct_cost * (parent.Missing_task_time - model.Possible_task_location_combination_workloads[(*l_it).Id][c] + transportation_times[(*l_it).Id][number_locations - 1]) < vehicle.Max_cost - PRECISION) {
                                        Candidate extension_candidate;
                                        extension_candidate.Id = cand_id;
                                        extension_candidate.Extension = {(*l_it).Id, c, start_time};
                                        extension_candidate.Add_cost = additional_cost;
                                        extension_candidate.End_time = end_time;
                                        extension_candidate.Level_index = (*l_it).Level_index;
                                        extension_candidate.Visit_ZHC_Change = visit_extend;
                                        candidates.push_back(extension_candidate);
                                        ++cand_id;
                                }
                                if (cum_block_cost == 0 && cum_cut_cost == 0)
                                    break;
                            }
                        }
                        ++waiting;
                    }
                }
            }
        }
        time_sp_identify_extension += double(clock() - start_sp_identify_extension) / CLOCKS_PER_SEC;
        // extend route fragment by candidates
        for (size_t c_index = 0; c_index != candidates.size(); ++c_index) {
            clock_t start_sp_perform_extension = clock();
            Candidate cand(candidates[c_index]);
            Route_fragment new_fragment;
            new_fragment.Route = parent.Route;
            new_fragment.Route.push_back(cand.Extension);
            new_fragment.Loc = cand.Extension[0];
            new_fragment.End_time = cand.End_time;
            new_fragment.Level_index = cand.Level_index;
            new_fragment.Cost = parent.Cost + cand.Add_cost;
            new_fragment.Missing_task_time = parent.Missing_task_time - model.Possible_task_location_combination_workloads[cand.Extension[0]][cand.Extension[1]];
            new_fragment.Missing_tasks = parent.Missing_tasks;
            for (auto t_it = model.Possible_task_location_combinations_list[cand.Extension[0]][cand.Extension[1]].begin(); t_it != model.Possible_task_location_combinations_list[cand.Extension[0]][cand.Extension[1]].end(); ++t_it) {
                new_fragment.Missing_tasks[*t_it] = false;
            }
            if (new_fragment.Loc == number_locations - 1) {
                if (new_fragment.Missing_tasks == all_tasks_performed) { // this condition can be removed
                    new_routes.push_back(new_fragment.Route);
                    new_routes_cost.push_back(new_fragment.Cost);
                }
                time_sp_perform_extension += double(clock() - start_sp_perform_extension) / CLOCKS_PER_SEC;
                continue;
            }
            new_fragment.Accessible_locations = parent.Accessible_locations;
            new_fragment.Accessible_locations[new_fragment.Loc] = false;
            for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
                if (!new_fragment.Accessible_locations[(*l_it).Id])
                    continue;
                if ((*l_it).Level_index < new_fragment.Level_index) {
                    new_fragment.Accessible_locations[(*l_it).Id] = false;
                    continue;
                }
            }
            new_fragment.Visits_ZHC = parent.Visits_ZHC;
            for (int c = 0; c != zero_half_cuts.size(); ++c) {
                new_fragment.Visits_ZHC[c] += cand.Visit_ZHC_Change[c];
            }
            time_sp_perform_extension += double(clock() - start_sp_perform_extension) / CLOCKS_PER_SEC;
            // dominance check 1: check if new route fragment is dominated by existing route fragment
            clock_t start_sp_dominance_check_1 = clock();
            bool not_dominated = check_dominance1(open_fragments, new_fragment, number_tasks, number_locations, vehicle.Sct_cost, cut_cost);
            time_sp_dominance_check_1 += double(clock() - start_sp_dominance_check_1) / CLOCKS_PER_SEC;
            if (not_dominated) {
                clock_t start_sp_dominance_check_2 = clock();
                // dominance check 2: check if new route fragment is dominating existing route fragment
                check_dominance2(open_fragments, new_fragment, number_tasks, number_locations, vehicle.Sct_cost, cut_cost);
                time_sp_dominance_check_2 += double(clock() - start_sp_dominance_check_2) / CLOCKS_PER_SEC;
                clock_t start_add_fragment = clock();
                new_fragment.Id = id_count;
                ++id_count;
                bool new_fragment_inserted(false);
                for (auto it = open_fragments.begin(); it != open_fragments.end(); ++it) {
                    if ((*it).Level_index > new_fragment.Level_index) {
                        open_fragments.insert(it, new_fragment);
                        new_fragment_inserted = true;
                        break;
                    }
                    if ((*it).Level_index == new_fragment.Level_index && (*it).Cost > new_fragment.Cost) {
                        open_fragments.insert(it, new_fragment);
                        new_fragment_inserted = true;
                        break;
                    }
                }
                if (!new_fragment_inserted)
                    open_fragments.push_back(new_fragment);
                time_add_fragment += double(clock() - start_add_fragment) / CLOCKS_PER_SEC;
            }
        }
    }
}
//------------------------------------------------------------------------------------------------------------//
