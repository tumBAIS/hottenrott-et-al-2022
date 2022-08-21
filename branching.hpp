//------------------------------------------------------------------------------------------------------------//
/* Derive new nodes based on branching.*/
//------------------------------------------------------------------------------------------------------------//
using namespace std;
//------------------------------------------------------------------------------------------------------------//
// Type definitions
typedef vector<int> VecI;
typedef vector<bool> VecB;
typedef vector<double> VecD;
typedef vector<string> VecS;
//------------------------------------------------------------------------------------------------------------//
void branch_on_assign(bool &found, vector<Node> &unsolved, VecI &branched_on, int &id_count, vector<Vehicle> &vehicles, vector<Location> &locations,
        int &number_locations, int &number_tasks,
        Node &parent_node, vector<vector<VecD>> &node_split_tasks, double &node_lb, int &v_id) {
    // find branching variable
    VecI branch_var;
    int preference_left(0);
    double value(-0.1); // branch on task assignment that is closest to 0 or 1
    for (int t = 1; t != number_tasks - 1; ++t) {
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            if ((*l_it).Id == 0 || (*l_it).Id == number_locations - 1)
                continue;
            if (node_split_tasks[v_id][(*l_it).Id][t] > PRECISION &&
                node_split_tasks[v_id][(*l_it).Id][t] < 1 - PRECISION) {
                if (abs(node_split_tasks[v_id][(*l_it).Id][t] - 0.5) > value) {
                    branch_var = {v_id, (*l_it).Id, t};
                    value = abs(node_split_tasks[v_id][(*l_it).Id][t] - 0.5);
                    found = true;
                    if (node_split_tasks[v_id][(*l_it).Id][t] >= 0.5) {
                        preference_left = 0;
                    }
                    else {
                        preference_left = 1;
                    }
                }
            }
        }
    }
    if (found) {
        cout << "BRANCH ON ASSIGN: V" << branch_var[0] << " L" << branch_var[1] << " T" << branch_var[2] << " " << parent_node.Depth << endl;
        ++branched_on[0];
        // create down branch (left): task not to be performed on location
        Node new_node1;
        new_node1.Id = id_count;
        ++id_count;
        new_node1.Lb = max(node_lb, parent_node.Lb);
        new_node1.Depth = parent_node.Depth + 1;
        new_node1.Preference = parent_node.Preference + preference_left;
        new_node1.Fixed_vehicle_location_tasks = parent_node.Fixed_vehicle_location_tasks;
        new_node1.Fixed_vehicle_location_tasks[branch_var[0]][branch_var[1]][branch_var[2]] = 0;
        new_node1.Enforced_vehicle_location_times = parent_node.Enforced_vehicle_location_times;
        new_node1.Forbidden_vehicle_location_times = parent_node.Forbidden_vehicle_location_times;
        new_node1.Task_start_times_lower_bounds = parent_node.Task_start_times_lower_bounds;
        new_node1.Task_start_times_upper_bounds = parent_node.Task_start_times_upper_bounds;
        unsolved.push_back(new_node1);
        // create up branch (right): task to be performed on location
        Node new_node2;
        new_node2.Id = id_count;
        ++id_count;
        new_node2.Lb = max(node_lb, parent_node.Lb);
        new_node2.Depth = parent_node.Depth + 1;
        new_node2.Preference = parent_node.Preference + preference_left;
        new_node2.Fixed_vehicle_location_tasks = parent_node.Fixed_vehicle_location_tasks;
        new_node2.Fixed_vehicle_location_tasks[branch_var[0]][branch_var[1]][branch_var[2]] = 1;
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            if ((*l_it).Id != branch_var[1] && (*l_it).Assignments[branch_var[2]]) {
                new_node2.Fixed_vehicle_location_tasks[branch_var[0]][(*l_it).Id][branch_var[2]] = 0;
            }
        }
        new_node2.Enforced_vehicle_location_times = parent_node.Enforced_vehicle_location_times;
        new_node2.Forbidden_vehicle_location_times = parent_node.Forbidden_vehicle_location_times;
        new_node2.Task_start_times_lower_bounds = parent_node.Task_start_times_lower_bounds;
        new_node2.Task_start_times_upper_bounds = parent_node.Task_start_times_upper_bounds;
        unsolved.push_back(new_node2);
    }
}
//------------------------------------------------------------------------------------------------------------//
void branch_on_usage(bool &found, vector<Node> &unsolved, VecI &branched_on, int &id_count, vector<Vehicle> &vehicles, vector<Location> &locations,
        int &number_locations, int &horizon, Node &parent_node, vector<vector<VecD>> &node_split_usage, double &node_lb, int &v_id) {
    // find branching variable
    VecI branch_var;
    double value(-0.1);
    int preference_left(0);
    for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
        if ((*l_it).Id == 0 || (*l_it).Id == number_locations - 1)
            continue;
        for (int z = 0; z != horizon; ++z) {
            if (node_split_usage[v_id][(*l_it).Id][z] > PRECISION && node_split_usage[v_id][(*l_it).Id][z] < 1 - PRECISION) {
                if (abs(node_split_usage[v_id][(*l_it).Id][z] - 0.5) > value) {
                    branch_var = {v_id, (*l_it).Id, z};
                    found = true;
                    value = abs(node_split_usage[v_id][(*l_it).Id][z] - 0.5);
                    if (node_split_usage[v_id][(*l_it).Id][z] > 0) {
                        preference_left = 0;
                    }
                    else {
                        preference_left = 1;
                    }
                }
            }
        }
    }
    if (found) {
        cout << "BRANCH ON USAGE: V" << branch_var[0] << " L" << branch_var[1] << " Z" << branch_var[2] << " " << node_split_usage[branch_var[0]][branch_var[1]][branch_var[2]] << " " << parent_node.Depth <<  endl;
        ++branched_on[1];
        // create down branch (left): vehicle does not block location at time z
        Node new_node1;
        new_node1.Id = id_count;
        ++id_count;
        new_node1.Lb = max(node_lb, parent_node.Lb);
        new_node1.Depth = parent_node.Depth + 1;
        new_node1.Preference = parent_node.Preference + preference_left;
        new_node1.Fixed_vehicle_location_tasks = parent_node.Fixed_vehicle_location_tasks;
        new_node1.Enforced_vehicle_location_times = parent_node.Enforced_vehicle_location_times;
        new_node1.Forbidden_vehicle_location_times = parent_node.Forbidden_vehicle_location_times;
        new_node1.Forbidden_vehicle_location_times[branch_var[0]][branch_var[1]].push_back(branch_var[2]);
        new_node1.Task_start_times_lower_bounds = parent_node.Task_start_times_lower_bounds;
        new_node1.Task_start_times_upper_bounds = parent_node.Task_start_times_upper_bounds;
        unsolved.push_back(new_node1);
        // create up branch (right): vehicle blocks location at time z
        Node new_node2;
        new_node2.Id = id_count;
        ++id_count;
        new_node2.Lb = max(node_lb, parent_node.Lb);
        new_node2.Depth = parent_node.Depth + 1;
        new_node2.Preference = parent_node.Preference + (1 - preference_left);
        new_node2.Fixed_vehicle_location_tasks = parent_node.Fixed_vehicle_location_tasks;
        new_node2.Enforced_vehicle_location_times = parent_node.Enforced_vehicle_location_times;
        new_node2.Forbidden_vehicle_location_times = parent_node.Forbidden_vehicle_location_times;
        new_node2.Enforced_vehicle_location_times[branch_var[0]][branch_var[1]].push_back(branch_var[2]);
        for (auto l_it = locations.begin(); l_it != locations.end(); ++l_it) {
            if ((*l_it).Id != branch_var[1]) {
                new_node2.Forbidden_vehicle_location_times[branch_var[0]][(*l_it).Id].push_back(branch_var[2]);
            }
        }
        for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
            if ((*v_it).Id != branch_var[0]) {
                new_node2.Forbidden_vehicle_location_times[(*v_it).Id][branch_var[1]].push_back(branch_var[2]);
            }
        }
        new_node2.Task_start_times_lower_bounds = parent_node.Task_start_times_lower_bounds;
        new_node2.Task_start_times_upper_bounds = parent_node.Task_start_times_upper_bounds;
        unsolved.push_back(new_node2);
    }
}
//------------------------------------------------------------------------------------------------------------//
void branch_on_time(bool &found, vector<Node> &unsolved, VecI &branched_on, int &id_count, vector<Model> &models, vector<Vehicle> &vehicles, vector<Location> &locations,
        int &number_locations, int &number_tasks, int &horizon,
        Node &parent_node, vector<vector<VecD>> &node_split_times, double &node_lb, int &v_id) {
    // find branching variable
    VecI branch_var;
    int min_split_range(horizon + 1);
    for (int t = 1; t != number_tasks - 1; ++t) {
        bool earliest_found(false);
        int earliest(0);
        int latest(0);
        for (int z = 0; z != horizon; ++z) {
            if (node_split_times[v_id][t][z] > PRECISION && node_split_times[v_id][t][z] < 1 - PRECISION) {
                if (!earliest_found) {
                    earliest = z;
                    earliest_found = true;
                }
                latest = z;
            }
        }
        if (latest - earliest >= 1 && latest - earliest < min_split_range) {
            branch_var = {v_id, t, int((latest + earliest) / 2)};
            found = true;
            min_split_range = latest - earliest;
        }
    }
    if (found) {
        // determine preference
        int preference_left(1);
        double sum_lower(0.0);
        for (int z = 0; z != branch_var[2] + 1; ++z) {
            sum_lower += node_split_times[branch_var[0]][branch_var[1]][z];
        }
        if (sum_lower < 0.5) {
            preference_left = 0;
        }
        cout << "BRANCH ON TIME: V" << branch_var[0] << " T" << branch_var[1] << " Z" << branch_var[2] << " " << parent_node.Depth << endl;
        ++branched_on[2];
        Model model(*(models.begin() + (*(vehicles.begin() + branch_var[0])).Type));
        // create down branch (left): task has to be performed earlier than branch time
        Node new_node1;
        new_node1.Id = id_count;
        ++id_count;
        new_node1.Lb = max(node_lb, parent_node.Lb);
        new_node1.Depth = parent_node.Depth + 1;
        new_node1.Preference = parent_node.Preference + preference_left;
        new_node1.Fixed_vehicle_location_tasks = parent_node.Fixed_vehicle_location_tasks;
        new_node1.Enforced_vehicle_location_times = parent_node.Enforced_vehicle_location_times;
        new_node1.Forbidden_vehicle_location_times = parent_node.Forbidden_vehicle_location_times;
        new_node1.Task_start_times_lower_bounds = parent_node.Task_start_times_lower_bounds;
        new_node1.Task_start_times_upper_bounds = parent_node.Task_start_times_upper_bounds;
        new_node1.Task_start_times_upper_bounds[branch_var[0]][branch_var[1]] = branch_var[2];
        unsolved.push_back(new_node1);
        // create up branch (right): task has to be performed later or equal than branch time
        Node new_node2;
        new_node2.Id = id_count;
        ++id_count;
        new_node2.Lb = max(node_lb, parent_node.Lb);
        new_node2.Depth = parent_node.Depth + 1;
        new_node2.Preference = parent_node.Preference + (1 - preference_left);
        new_node2.Fixed_vehicle_location_tasks = parent_node.Fixed_vehicle_location_tasks;
        new_node2.Enforced_vehicle_location_times = parent_node.Enforced_vehicle_location_times;
        new_node2.Forbidden_vehicle_location_times = parent_node.Forbidden_vehicle_location_times;
        new_node2.Task_start_times_lower_bounds = parent_node.Task_start_times_lower_bounds;
        new_node2.Task_start_times_upper_bounds = parent_node.Task_start_times_upper_bounds;
        new_node2.Task_start_times_lower_bounds[branch_var[0]][branch_var[1]] = branch_var[2] + 1;
        unsolved.push_back(new_node2);
    }
}
//------------------------------------------------------------------------------------------------------------//