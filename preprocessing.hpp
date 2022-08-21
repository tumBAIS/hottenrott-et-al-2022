//------------------------------------------------------------------------------------------------------------//
/* Preprocessing for given instance.
 * Determine possible task-location combinations for all models and corresponding workloads.
 * Determine preselect routes for all models.*/
//------------------------------------------------------------------------------------------------------------//
using namespace std;
//------------------------------------------------------------------------------------------------------------//
// Type definitions
typedef vector<int> VecI;
typedef vector<bool> VecB;
typedef vector<double> VecD;
typedef vector<string> VecS;
//------------------------------------------------------------------------------------------------------------//
void determine_combinations_util(vector<VecI>& ans, VecI& tmp, size_t n, int left, size_t k) {
    // Pushing this vector to a vector of vector
    if (k == 0) {
        ans.push_back(tmp);
        return;
    }
    // i iterates from left to n. First time
    // left will be 1
    for (int i = left; i <= n; ++i) {
        tmp.push_back(i);
        determine_combinations_util(ans, tmp, n, i + 1, k - 1);
        // Popping out last inserted element
        // from the vector
        tmp.pop_back();
    }
}
//------------------------------------------------------------------------------------------------------------//
vector<VecI> determine_combinations(size_t n, size_t k) {
    vector<VecI> ans;
    VecI tmp;
    determine_combinations_util(ans, tmp, n, 0, k);
    return ans;
}
//------------------------------------------------------------------------------------------------------------//
/* Determine possible task-location combinations for all models and corresponding workloads.
 * Determine preselect routes for all models. */
void preprocess(vector<Model> &models, vector<Location> &locations, int &number_locations, int &number_tasks,
        Location &start_loc, Location &end_loc) {
    for (auto m_it = models.begin(); m_it != models.end(); ++m_it) {
        vector<vector<VecB>> possible_tlc;
        vector<vector<VecI>> possible_tlc_list;
        vector<VecI> possible_tlc_workloads;
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            VecI tasks_at_location;
            for (int t = 0; t != number_tasks; ++t) {
                if ((*m_it).Tasks_per_model[t] && (*l_it).Assignments[t]) {
                    tasks_at_location.push_back(t);
                }
            }
            // derive all possible combinations of tasks at location for model
            vector<VecB> combinations;
            vector<VecI> combinations_list;
            VecI workloads;
            for (size_t number = 1; number <= tasks_at_location.size(); ++number) {
                vector<VecI> comb = determine_combinations(tasks_at_location.size() - 1, number);
                for (size_t c = 0; c != comb.size(); ++c) {
                    VecI covered_tasks;
                    for (size_t t_index = 0; t_index != comb[c].size(); ++t_index) {
                        covered_tasks.push_back(tasks_at_location[comb[c][t_index]]);
                    }
                    // eliminate task combinations if a task that has to be carried out in the middle is not part of the set
                    bool feasible(true);
                    VecI combined_predecessors;
                    VecI combined_successors;
                    for (int t = 0; t != number_tasks; ++t) {
                        if (find(covered_tasks.begin(), covered_tasks.end(), t) != covered_tasks.end()) {
                            for (int t2 = 0; t2 != number_tasks; ++t2) {
                                if (find(covered_tasks.begin(), covered_tasks.end(), t2) == covered_tasks.end()) {
                                    if ((*m_it).All_successors[t][t2]) {
                                        if (find(combined_successors.begin(), combined_successors.end(), t2) ==
                                            combined_successors.end()) {
                                            combined_successors.push_back(t2);
                                        }
                                    } else if ((*m_it).All_predecessors[t][t2]) {
                                        if (find(combined_predecessors.begin(), combined_predecessors.end(), t2) ==
                                            combined_predecessors.end()) {
                                            combined_predecessors.push_back(t2);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    for (int t2 = 0; t2 != number_tasks; ++t2) {
                        if ((find(combined_successors.begin(), combined_successors.end(), t2) !=
                             combined_successors.end()) &&
                            (find(combined_predecessors.begin(), combined_predecessors.end(), t2) !=
                             combined_predecessors.end())) {
                            feasible = false;
                            break;
                        }
                    }
                    if (!feasible)
                        continue;
                    VecB comb_bool;
                    VecI comb_list;
                    for (int t = 0; t != number_tasks; ++t) {
                        if (find(covered_tasks.begin(), covered_tasks.end(), t) != covered_tasks.end()) {
                            comb_bool.push_back(true);
                            comb_list.push_back(t);
                        } else {
                            comb_bool.push_back(false);
                        }
                    }
                    combinations.push_back(comb_bool);
                    combinations_list.push_back(comb_list);
                    int workload(0);
                    for (size_t t = 0; t != number_tasks; ++t) {
                        if (comb_bool[t])
                            workload += (*m_it).Task_times[t];
                    }
                    workloads.push_back(workload);
                }
            }
            possible_tlc.push_back(combinations);
            possible_tlc_list.push_back(combinations_list);
            possible_tlc_workloads.push_back(workloads);
        }
        (*m_it).Possible_task_location_combinations = possible_tlc;
        (*m_it).Possible_task_location_combinations_list = possible_tlc_list;
        (*m_it).Possible_task_location_combination_workloads = possible_tlc_workloads;
        vector<VecI> preselect_route;
        VecI last({start_loc.Id, 0});
        int last_level_index(start_loc.Level_index);
        int last_row_index(start_loc.Row_index);
        preselect_route.push_back(last);
        VecB covered_tasks(number_tasks, false);
        covered_tasks[0] = true;
        while (1) {
            // finde next loc-task combination
            bool found(false);
            for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
                if (last[0] == (*l_it).Id || (*l_it).Level_index < last_level_index)
                    continue;
                for (size_t c = 0; c != (*m_it).Possible_task_location_combinations[(*l_it).Id].size(); ++c) {
                    if ((*m_it).Possible_task_location_combinations[(*l_it).Id][c] ==
                        (*m_it).Preselect_assignments[(*l_it).Id]) {
                        bool feasible(true);
                        for (int t = 0; t != number_tasks; ++t) {
                            if ((*m_it).Possible_task_location_combinations[(*l_it).Id][c][t]) {
                                if (covered_tasks[t]) {
                                    feasible = false;
                                    break;
                                }
                                for (int tp = 0; tp != number_tasks; ++tp) {
                                    if ((*m_it).Preselect_all_predecessors[t][tp]) {
                                        if (!covered_tasks[tp] &&
                                            !(*m_it).Possible_task_location_combinations[(*l_it).Id][c][tp]) {
                                            feasible = false;
                                            break;
                                        }
                                    }
                                }
                                if (!feasible)
                                    break;
                            }
                        }
                        if (feasible) {
                            VecI next({(int) (*l_it).Id, (int) c});
                            preselect_route.push_back(next);
                            for (int t = 0; t != number_tasks; ++t) {
                                if ((*m_it).Possible_task_location_combinations[(*l_it).Id][c][t]) {
                                    covered_tasks[t] = true;
                                }
                            }
                            last = next;
                            last_level_index = (*l_it).Level_index;
                            last_row_index = (*l_it).Row_index;
                            found = true;
                            break;
                        }
                    }
                }
                if (found)
                    break;
            }
            if (covered_tasks == (*m_it).Tasks_per_model)
                break;
        }
        (*m_it).Preselect_route = preselect_route;
        VecI route_workloads(number_locations, 0);
        for (auto r_it = preselect_route.begin(); r_it != preselect_route.end(); ++r_it) {
            route_workloads[(*r_it)[0]] += (*m_it).Possible_task_location_combination_workloads[(*r_it)[0]][(*r_it)[1]];
        }
        (*m_it).Preselect_route_workloads = route_workloads;
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Initialize blockings data structure. Blockings show which vehicle-routes block a location at certain point in time.*/
void initialize_blockings(vector<vector<vector<VecI>>> &blockings, vector<Location> &locations, vector<Vehicle> &vehicles, int &horizon) {
    vector<vector<vector<VecI>>> new_blockings;
    for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
        vector<vector<VecI>> blockings_l;
        for (int z = 0; z != horizon; ++z) {
            blockings_l.push_back({});
        }
        new_blockings.push_back(blockings_l);
    }
    blockings = new_blockings;
    // include blockings of routes in initial solution
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            for (int z = (*v_it).Route_start_times[0][(*l_it).Id]; z != (*v_it).Route_end_times[0][(*l_it).Id]; ++z) {
                blockings[(*l_it).Id][z].push_back({(*v_it).Id, 0});
            }
        }
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Check if it worth investigating OF scenario. In case the locations on the same level cannot be permutated, no potential. */
void check_of_potential(bool &of_has_potential, vector<Model> &models, vector<Location> &locations, int &number_locations, int &number_tasks) {
    for (auto m_it = models.begin(); m_it != models.end(); ++m_it) {
        VecI visited_loc;
        for (auto r_it = (*m_it).Preselect_route.begin(); r_it != (*m_it).Preselect_route.end(); ++r_it) {
            visited_loc.push_back((*r_it)[0]);
        }
        for (auto l1_it = visited_loc.begin(); l1_it != visited_loc.end(); ++l1_it) {
            for (auto l2_it = visited_loc.begin(); l2_it != visited_loc.end(); ++l2_it) {
                if (*l1_it < *l2_it && locations[*l1_it].Level_index == locations[*l2_it].Level_index) {
                    bool precedence(false);
                    for (auto t1_it = (*m_it).Possible_task_location_combinations_list[*l1_it][(*m_it).Scenario_task_location_combinations[*l1_it][0]].begin(); t1_it != (*m_it).Possible_task_location_combinations_list[*l1_it][(*m_it).Scenario_task_location_combinations[*l1_it][0]].end(); ++t1_it) {
                        for (auto t2_it = (*m_it).Possible_task_location_combinations_list[*l2_it][(*m_it).Scenario_task_location_combinations[*l2_it][0]].begin(); t2_it != (*m_it).Possible_task_location_combinations_list[*l2_it][(*m_it).Scenario_task_location_combinations[*l2_it][0]].end(); ++t2_it) {
                            if ((*m_it).Scenario_all_predecessors[*t1_it][*t2_it] || (*m_it).Scenario_all_successors[*t1_it][*t2_it]) {
                                precedence = true;
                                break;
                            }
                        }
                        if (precedence)
                            break;
                    }
                    if (!precedence) {
                        of_has_potential = true;
                        return;
                    }
                }
            }
        }
    }
}
//------------------------------------------------------------------------------------------------------------//

