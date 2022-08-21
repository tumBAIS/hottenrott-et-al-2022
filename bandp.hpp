//------------------------------------------------------------------------------------------------------------//
/* Solve for given flexibility scenario using branch-and-price algorithm.*/
//------------------------------------------------------------------------------------------------------------//
using namespace std;
//------------------------------------------------------------------------------------------------------------//
// Type definitions
typedef vector<int> VecI;
typedef vector<bool> VecB;
typedef vector<double> VecD;
typedef vector<string> VecS;
//------------------------------------------------------------------------------------------------------------//
/* Initialize root node for scenarios. */
void root_node(vector<vector<VecI>> &fixed_vlt, vector<vector<VecI>> &enforced_vlz, vector<vector<VecI>> &forbidden_vlz, vector<VecI> &start_lb, vector<VecI> &start_ub,
        vector<Model> &models, vector<Vehicle> &vehicles, vector<Location> &locations, int &number_locations, int &number_tasks, int &horizon) {
    // initialize fixed vlt
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        VecI dummy_vector(number_tasks, -1);
        vector<VecI> location_task_assignments(number_locations, dummy_vector);
        fixed_vlt.push_back(location_task_assignments);
    }
    // initialize enforced and forbidden vlz
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        VecI dummy({});
        vector<VecI> enforced_v(number_locations, dummy);
        vector<VecI> forbidden_v(number_locations, dummy);
        enforced_vlz.push_back(enforced_v);
        forbidden_vlz.push_back(forbidden_v);
    }
    // initialize lower bounds on starting times of tasks
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        VecI start_lb_v(number_tasks, 0);
        start_lb.push_back(start_lb_v);
    }
    // initialize upper bounds on starting times of tasks
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        VecI start_ub_v(number_tasks, horizon);
        start_ub.push_back(start_ub_v);
    }
}
//------------------------------------------------------------------------------------------------------------//
/* Set data entries for root node. */
void initialize_root_node(Node &root, vector<vector<VecI>> &fixed_vlt, vector<vector<VecI>> &enforced_vlz,
        vector<vector<VecI>> &forbidden_vlz, vector<VecI> &start_lb, vector<VecI> &start_ub, vector<Model> &models, vector<Vehicle> &vehicles) {
    root.Id = 0;
    int max_min_sct(0);
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        if (models[(*v_it).Type].Min_sct > max_min_sct) {
            max_min_sct = models[(*v_it).Type].Min_sct;
        }
    }
    root.Lb = max_min_sct;
    root.Depth = 0;
    root.Preference = 0;
    root.Fixed_vehicle_location_tasks = fixed_vlt;
    root.Enforced_vehicle_location_times = enforced_vlz;
    root.Forbidden_vehicle_location_times = forbidden_vlz;
    root.Task_start_times_lower_bounds = start_lb;
    root.Task_start_times_upper_bounds = start_ub;
}
//------------------------------------------------------------------------------------------------------------//
double solve_root_node(int &scenario, vector<Model> &models, vector<Vehicle> &vehicles, vector<Location> &locations,
                     int &number_locations, int &number_tasks, vector<vector<vector<VecI>>> &blockings, vector<VecI> &transportation_times,
                     VecI &best_sol, int &horizon, int &ub, clock_t &start_scenario,
                     double &time_limit, bool &master_activity_end, int &max_routes_per_vehicle) {
    double time_create_mp(0.0), time_solve_mp(0.0), time_identify_cuts(0.0), time_read_duals(0.0), time_solve_sp(0.0), time_add_columns(0.0), time_mip_search(0.0), time_branch(0.0), time_preprocess_node(0.0), time_postprocess_node(0.0), time_cutoff(0.0), time_sort_node(0.0), time_solve_node(0.0), time_check_integrality_and_update_best(0.0);
    double time_sp_identify_extension(0.0), time_sp_perform_extension(0.0), time_sp_dominance_check_1(0.0), time_sp_dominance_check_2(0.0), time_add_fragment(0.0);
    // initialize root node
    vector<vector<VecI>> fixed_vlt;
    vector<vector<VecI>> enforced_vlz;
    vector<vector<VecI>> forbidden_vlz;
    vector<VecI> start_lb, start_ub;
    root_node(fixed_vlt, enforced_vlz, forbidden_vlz, start_lb, start_ub, models, vehicles, locations, number_locations, number_tasks, horizon);
    Node root;
    initialize_root_node(root, fixed_vlt, enforced_vlz, forbidden_vlz, start_lb, start_ub, models, vehicles);
    double node_lb(ub);
    bool node_is_integral(true);
    bool node_is_feasible(true);
    vector<VecD> node_split_routes;
    vector<vector<VecD>> node_split_usage;
    vector<vector<VecD>> node_split_tasks;
    vector<vector<VecD>> node_split_times;
    int no_ub_escape(0);
    solve_node(horizon, ub, no_ub_escape, best_sol, node_lb, node_is_integral, node_is_feasible, node_split_routes, node_split_usage, node_split_tasks, node_split_times,
               root, models, vehicles, locations, blockings, transportation_times, number_tasks, number_locations, master_activity_end, max_routes_per_vehicle, time_limit, start_scenario, time_create_mp, time_solve_mp, time_identify_cuts, time_read_duals,
               time_solve_sp, time_add_columns, time_mip_search, time_sp_identify_extension, time_sp_perform_extension, time_sp_dominance_check_1, time_sp_dominance_check_2, time_add_fragment, time_preprocess_node, time_postprocess_node, time_check_integrality_and_update_best);
    return node_lb;
}
//------------------------------------------------------------------------------------------------------------//
/* Solve branch-and-price.*/
void solve_branch_and_price(int &scenario, vector<Model> &models, vector<Vehicle> &vehicles, vector<Location> &locations,
        int &number_locations, int &number_tasks, vector<vector<vector<VecI>>> &blockings, vector<VecI> &transportation_times,
        VecI &best_sol, int &horizon, int &ub, int &ub_escape, clock_t &start_scenario,
        double &time_limit, bool &master_activity_end, VecS &branching, int &max_routes_per_vehicle) {
    clock_t start_bandp = clock();
    double time_create_mp(0.0), time_solve_mp(0.0), time_identify_cuts(0.0), time_read_duals(0.0), time_solve_sp(0.0), time_add_columns(0.0), time_mip_search(0.0), time_branch(0.0), time_preprocess_node(0.0), time_postprocess_node(0.0), time_cutoff(0.0), time_sort_node(0.0), time_solve_node(0.0), time_check_integrality_and_update_best(0.0);
    double time_sp_identify_extension(0.0), time_sp_perform_extension(0.0), time_sp_dominance_check_1(0.0), time_sp_dominance_check_2(0.0), time_add_fragment(0.0);
    // initialize root node
    vector<vector<VecI>> fixed_vlt;
    vector<vector<VecI>> enforced_vlz;
    vector<vector<VecI>> forbidden_vlz;
    vector<VecI> start_lb, start_ub;
    root_node(fixed_vlt, enforced_vlz, forbidden_vlz, start_lb, start_ub, models, vehicles, locations, number_locations, number_tasks, horizon);
    Node root;
    initialize_root_node(root, fixed_vlt, enforced_vlz, forbidden_vlz, start_lb, start_ub, models, vehicles);
    // initialize tree
    vector<Node> unsolved({});
    unsolved.push_back(root);
    int explored_nodes(0);
    int id_count(1);
    double global_lb(root.Lb);
    // start branch-and-bound tree
    VecI branched_on = {0,0,0};
    while (1) {
        // select first node from unsolved list as parent node
        Node parent_node(unsolved[0]);
        // erase parent node
        unsolved.erase(unsolved.begin());
        // solve parent node LP relaxation using column generation
        double node_lb(ub);
        bool node_is_integral(true);
        bool node_is_feasible(true);
        vector<VecD> node_split_routes;
        vector<vector<VecD>> node_split_usage;
        vector<vector<VecD>> node_split_tasks;
        vector<vector<VecD>> node_split_times;
        clock_t start_solve_node = clock();
        solve_node(horizon, ub, ub_escape, best_sol, node_lb, node_is_integral, node_is_feasible, node_split_routes, node_split_usage, node_split_tasks, node_split_times,
                parent_node, models, vehicles, locations, blockings, transportation_times, number_tasks, number_locations, master_activity_end, max_routes_per_vehicle, time_limit, start_scenario, time_create_mp, time_solve_mp, time_identify_cuts, time_read_duals,
                time_solve_sp, time_add_columns, time_mip_search, time_sp_identify_extension, time_sp_perform_extension, time_sp_dominance_check_1, time_sp_dominance_check_2, time_add_fragment, time_preprocess_node, time_postprocess_node, time_check_integrality_and_update_best);
        time_solve_node += double(clock() - start_solve_node) / CLOCKS_PER_SEC;
        if (double(double(clock() - start_scenario) / CLOCKS_PER_SEC) >= time_limit) {
            return;
        }
        if (ub <= ub_escape) {
            break;
        }
        if (node_is_feasible && !node_is_integral && ub - node_lb > 1 - PRECISION) {
            // start branching
            clock_t start_branch = clock();
            vector<Vehicle> sorted_vehicles(vehicles);
            sort(sorted_vehicles.begin(), sorted_vehicles.end(), [](const auto &v1, const auto &v2) {return v1.Id < v2.Id;});
            sort(sorted_vehicles.begin(), sorted_vehicles.end(), [](const auto &v1, const auto &v2) {return v1.Current_sct > v2.Current_sct;});
            bool found(false);
            for (auto v_it = sorted_vehicles.begin(); v_it != sorted_vehicles.end(); ++v_it) {
                for (auto it = branching.begin(); it != branching.end(); ++it) {
                    if (*it == "ASSIGN") {
                        branch_on_assign(found, unsolved, branched_on, id_count, vehicles, locations, number_locations,
                                         number_tasks, parent_node, node_split_tasks, node_lb, (*v_it).Id);
                    } else if (*it == "USAGE") {
                        branch_on_usage(found, unsolved, branched_on, id_count, vehicles, locations, number_locations,
                                        horizon, parent_node, node_split_usage, node_lb, (*v_it).Id);
                    } else if (*it == "TIME") {
                        branch_on_time(found, unsolved, branched_on, id_count, models, vehicles, locations,
                                       number_locations, number_tasks, horizon, parent_node, node_split_times, node_lb,
                                       (*v_it).Id);

                    }
                    if (found)
                        break;
                }
                if (found)
                    break;
            }
            time_branch += double(clock() - start_branch) / CLOCKS_PER_SEC;
        }
        // cut off dominated nodes in B&B tree
        clock_t start_cutoff = clock();
        auto it = unsolved.begin();
        while (it != unsolved.end()) {
            if (ub - (*it).Lb < 1 - PRECISION)
                unsolved.erase(it);
            else
                ++it;
        }
        time_cutoff += double(clock() - start_cutoff) / CLOCKS_PER_SEC;
        if (!unsolved.empty()) {
            // sort unsolved nodes
            clock_t start_sort_node = clock();
            sort(unsolved.begin(), unsolved.end(), [](const auto &n1, const auto &n2) {return n1.Id < n2.Id;});
            sort(unsolved.begin(), unsolved.end(), [](const auto &n1, const auto &n2) {return n1.Lb < n2.Lb;});
            global_lb = unsolved[0].Lb;
            sort(unsolved.begin(), unsolved.end(), [](const auto &n1, const auto &n2) {return n1.Preference > n2.Preference;});
            sort(unsolved.begin(), unsolved.end(), [](const auto &n1, const auto &n2) {return n1.Depth > n2.Depth;});
            time_sort_node += double(clock() - start_sort_node) / CLOCKS_PER_SEC;
            ++explored_nodes;
            cout << "EXPLORED: " << explored_nodes << " OPEN: " << unsolved.size() << " LB: " << global_lb << " UB: " << ub << " GAP: " << (ub - global_lb) / ub * 100 << "%" << endl;
        }
        else {
            global_lb = ub;
            ++explored_nodes;
            cout << "EXPLORED: " << explored_nodes << " OPEN: " << unsolved.size() << " LB: " << global_lb << " UB: " << ub << " GAP: " << (ub - global_lb) / ub * 100 << "%" << endl;
            break;
        }
    }
    double bandp_time = double(clock() - start_bandp) / CLOCKS_PER_SEC;
    cout << "-------------------------------------------------------------" << endl;
    cout << "Optimal solution with SCT " << ub << " found after evauating " << explored_nodes << " nodes in " << bandp_time << " seconds." << endl;
    cout << "-------------------------------------------------------------" << endl;
    cout << "Time to solve nodes: " << time_solve_node << " seconds." << endl;
    cout << "Time to create branches: " << time_branch << " seconds." << endl;
    cout << "Time to cutoff nodes: " << time_cutoff << " seconds." << endl;
    cout << "Time to sort nodes: " << time_sort_node << " seconds." << endl;
    cout << "-------------------------------------------------------------" << endl;
    cout << "Time to preprocess nodes: " << time_preprocess_node << " seconds." << endl;
    cout << "Time to create master problem: " << time_create_mp << " seconds." << endl;
    cout << "Time to solve master problem: " << time_solve_mp << " seconds." << endl;
    cout << "Time to identify cuts: " << time_identify_cuts << " seconds." << endl;
    cout << "Time to check integrality and update best: " << time_check_integrality_and_update_best << " seconds." << endl;
    cout << "Time to read dual values: " << time_read_duals << " seconds." << endl;
    cout << "Time to solve subproblems: " << time_solve_sp << " seconds." << endl;
    cout << "Time to add columns: " << time_add_columns << " seconds." << endl;
    cout << "Time to postprocess nodes: " << time_postprocess_node << " seconds." << endl;
    cout << "Time for MIP search: " << time_mip_search << " seconds." << endl;
    cout << "-------------------------------------------------------------" << endl;
    cout << "Time to identify candidates: " << time_sp_identify_extension << " seconds." << endl;
    cout << "Time for route extension: " << time_sp_perform_extension << " seconds." << endl;
    cout << "Time for dominance 1: " << time_sp_dominance_check_1 << " seconds." << endl;
    cout << "Time for dominance 2: " << time_sp_dominance_check_2 << " seconds." << endl;
    cout << "Time to add fragments: " << time_add_fragment << " seconds." << endl;
    cout << "-------------------------------------------------------------" << endl;
    cout << "Branched on: " << branched_on[0] << " " << branched_on[1] << " " << branched_on[2] << endl;
    cout << "-------------------------------------------------------------" << endl;
}
//------------------------------------------------------------------------------------------------------------//