//------------------------------------------------------------------------------------------------------------//
/* Create initial solution by scheduling all vehicles sequentially using NF routes. */
//------------------------------------------------------------------------------------------------------------//
using namespace std;
//------------------------------------------------------------------------------------------------------------//
// Type definitions
typedef vector<int> VecI;
typedef vector<bool> VecB;
typedef vector<double> VecD;
typedef vector<string> VecS;
//------------------------------------------------------------------------------------------------------------//
/* Heuristically generate initial solution using naive sequential scheduling approach. */
void generate_initial_solution(vector<Model> &models, vector<Vehicle> &vehicles, vector<Location> &locations,
        vector<VecI> &transportation_times, int &number_locations, int &ub) {
    VecI dummy({});
    vector<VecI> already_blocked(number_locations, dummy);
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        Model model(models[(*v_it).Type]);
        int time((*v_it).Arrival_time);
        int last_loc(0);
        vector<VecI> route;
        VecI start_times(number_locations, 0);
        VecI end_times(number_locations, 0);
        for (auto x_it = model.Preselect_route.begin(); x_it != model.Preselect_route.end(); ++x_it) {
            int start(time + transportation_times[last_loc][(*x_it)[0]]);
            while (1) {
                bool feasible(true);
                for (int z = start; z != start + model.Possible_task_location_combination_workloads[(*x_it)[0]][(*x_it)[1]]; ++z) {
                    if (find(already_blocked[(*x_it)[0]].begin(), already_blocked[(*x_it)[0]].end(), z) != already_blocked[(*x_it)[0]].end()) {
                        feasible = false;
                        break;
                    }
                }
                if (feasible) {
                    route.push_back({(*x_it)[0], (*x_it)[1], start});
                    time = start + model.Possible_task_location_combination_workloads[(*x_it)[0]][(*x_it)[1]];
                    start_times[(*x_it)[0]] = start;
                    end_times[(*x_it)[0]] = time;
                    last_loc = (*x_it)[0];
                    if ((*x_it)[0] != 0 && (*x_it)[0] != number_locations - 1) {
                        for (int z = start; z != time; ++z) {
                            already_blocked[(*x_it)[0]].push_back(z);
                        }
                    }
                    break;
                }
                ++start;
            }
        }
        (*v_it).All_routes.push_back(0);
        (*v_it).Route_details.push_back(route);
        (*v_it).Route_start_times.push_back(start_times);
        (*v_it).Route_end_times.push_back(end_times);
        (*v_it).Route_scts.push_back(time - (*v_it).Arrival_time);
        ub = max(ub, time - (*v_it).Arrival_time);
    }
}
//------------------------------------------------------------------------------------------------------------//