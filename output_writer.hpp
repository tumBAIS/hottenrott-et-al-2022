//------------------------------------------------------------------------------------------------------------//
/* Write results to csv files. */
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
void write_schedule_to_csv(string &name, vector<Model> &models, vector<Vehicle> &vehicles, vector<VecI> &transportation_times, VecI &best_sol, int &opt) {
    if (opt == -1)
        return;
    std::ofstream myfile;
    string filename("Results/Schedules/schedule_" + name + ".csv");
    myfile.open(filename);
    for (auto v_it = vehicles.begin(); v_it != vehicles.end(); ++v_it) {
        Model model(models[(*v_it).Type]);
        int last_loc(0);
        int last_time((*v_it).Arrival_time);
        for (auto x_it = (*v_it).Route_details[best_sol[(*v_it).Id]].begin(); x_it != (*v_it).Route_details[best_sol[(*v_it).Id]].end(); ++x_it) {
            if ((*x_it)[0] != last_loc) {
                myfile << (*v_it).Id << "," << model.Id << ", " << last_loc << "," << (*x_it)[0] << "," << -1 << "," << last_time << "," << last_time + transportation_times[last_loc][(*x_it)[0]] << "\n";
            }
            int time((*x_it)[2]);
            for (auto t_it = model.Possible_task_location_combinations_list[(*x_it)[0]][(*x_it)[1]].begin(); t_it != model.Possible_task_location_combinations_list[(*x_it)[0]][(*x_it)[1]].end(); ++t_it) {
                myfile << (*v_it).Id << "," << model.Id << ", " << (*x_it)[0] << "," << (*x_it)[0] << "," << (*t_it) << "," << time << "," << time + model.Task_times[(*t_it)] << "\n";
                time += model.Task_times[(*t_it)];
            }
            last_loc = (*x_it)[0];
            last_time = time;
        }
    }
    myfile.close();
}
//------------------------------------------------------------------------------------------------------------//
void write_solutions_to_csv(string &instance, VecI &sol_sct, VecD &sol_times, VecI &iterations) {
    std::ofstream myfile;
    string filename("Results/Solutions/solution_" + instance + ".csv");
    myfile.open(filename);
    for (auto it = sol_sct.begin(); it != sol_sct.end(); ++it) {
        myfile << (*it) << ",";
    }
    myfile << "\n";
    for (auto it = sol_times.begin(); it != sol_times.end(); ++it) {
        myfile << (*it) << ",";
    }
    myfile << "\n";
    for (auto it = iterations.begin(); it != iterations.end(); ++it) {
        myfile << (*it) << ",";
    }
    myfile.close();
}
//------------------------------------------------------------------------------------------------------------//

