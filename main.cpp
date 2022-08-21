//------------------------------------------------------------------------------------------------------------//
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <ctime>
#include "gurobi_c++.h"
#include <math.h>
#include "instance_list.hpp"
#include "instance_reader.hpp"
#include "preprocessing.hpp"
#include "preprocess_scenario.hpp"
#include "initial_solution.hpp"
#include "mip_ub_solver.hpp"
#include "route_finder.hpp"
#include "master_problem.hpp"
#include "aux_node.hpp"
#include "node_solver.hpp"
#include "branching.hpp"
#include "bandp.hpp"
#include "output_writer.hpp"
using namespace std;
//------------------------------------------------------------------------------------------------------------//
// type definitions
typedef vector<int> VecI;
typedef vector<bool> VecB;
typedef vector<double> VecD;
typedef vector<string> VecS;
//------------------------------------------------------------------------------------------------------------//
bool MIP_NF_SOLVE(true); // solve NF scenario using time-continuous MIP
bool MIP_OF_SOLVE(true); // solve OF scenario using time-continuous MIP
bool MASTER_ACTIVITY_END(true); // master problem: blocking constraints only at end of activities
bool USE_ALGORITHMIC_FRAMEWORK(true); // activate algorithmic framework
int MAX_ROUTES_PER_VEHICLE(50); // limit the maximum number of added routes per vehicle in each column generation iterations (the routes with the most negative reduced cost are chosen)
VecS BRANCHING{"ASSIGN", "TIME", "USAGE"}; // define sequence of branching choices
double TIME_LIMIT(3600.0); // time limit for solving a scenario
//-----------------------------------------------------------------------------------------------------------//
int main() {
    for (auto instance_it = INSTANCE_LIST.begin(); instance_it != INSTANCE_LIST.end(); ++instance_it) {
        cout << "--------------------------- Start ---------------------------" << endl;
        cout << "Start solving instance " << *instance_it << " with " << NUMBER_VEHICLES << " vehicles" << endl;
        clock_t start_instance = clock();
        //---------------------------------------------------------------------------------------------------//
        // define and read instance data from csv file
        clock_t start_read_data = clock();
        int number_vehicles(NUMBER_VEHICLES);
        int number_locations, number_tasks, number_models, cycle_time, total_workload;
        Location start_loc, end_loc;
        vector<Model> models;
        vector<Vehicle> vehicles;
        vector<Location> locations;
        vector<VecI> transportation_times;
        string filename(*instance_it + ".csv");
        read_instance(filename, number_models, number_vehicles, number_locations, number_tasks,
                      models, vehicles, locations, cycle_time, start_loc, end_loc, transportation_times, total_workload);
        double time_read_data = double(clock() - start_read_data) / CLOCKS_PER_SEC;
        //---------------------------------------------------------------------------------------------------//
        // preprocessing: generate possible model-task-location combinations and preselect routes (without timings)
        clock_t start_preprocessing = clock();
        preprocess(models, locations, number_locations, number_tasks, start_loc, end_loc);
        double time_preprocessing = double(clock() - start_preprocessing) / CLOCKS_PER_SEC;
        //---------------------------------------------------------------------------------------------------//
        // create upper bound by sequentially schedule all vehicles using NF routes
        int ub(-1);
        generate_initial_solution(models, vehicles, locations, transportation_times, number_locations, ub);
        cout << "Initial UB: " << ub << endl;
        cout << "-------------------------------------------------------------" << endl;
        int horizon(vehicles.back().Arrival_time + ub);
        VecI sol(number_vehicles, 0);
        string name = (*instance_it) + "_INIT";
        write_schedule_to_csv(name, models, vehicles, transportation_times, sol, ub);
        //---------------------------------------------------------------------------------------------------//
        // initialize blockings
        vector<vector<vector<VecI>>> blockings;
        initialize_blockings(blockings, locations, vehicles, horizon);
        //---------------------------------------------------------------------------------------------------//
        // initialize solution vectors: they show all optimal solutions for all scenarios and numbers of vehicles (we can use them as upper bounds for more flexible scenarios)
        VecI scenario_horizon(4, horizon);
        VecI sol_sct(4, ub);
        VecD sol_times(4, 0.0);
        VecI iterations(4, 0);
        vector<VecI> sol_routes(4, sol);
        // solve all scenarios
        for (int scenario = 0; scenario != 4; ++scenario) {
            clock_t start_scenario = clock();
            string string_scenario("ERROR");
            initialize_models_for_scenario(models, locations, number_locations, scenario, string_scenario);
            cout << "Start solving " << string_scenario << " scenario" << endl;
            if (scenario == 0 && MIP_NF_SOLVE) {
                bool warmstart(true);
                bool nf(true);
                solve_timecontinuous_mip_ub(sol_sct[scenario], sol_routes[scenario], models, vehicles, locations,
                                            transportation_times, number_locations, sol_routes[scenario],
                                            blockings, TIME_LIMIT, start_scenario, warmstart, nf);
            }
            else {
                // check if OF potential
                if (scenario == 1) {
                    bool of_has_potential(false);
                    check_of_potential(of_has_potential, models, locations, number_locations, number_tasks);
                    if (!of_has_potential) {
                        string name = (*instance_it) + "_" + string_scenario;
                        write_schedule_to_csv(name, models, vehicles, transportation_times, sol_routes[scenario], sol_sct[scenario]);
                        cout << "OF has no potential!" << endl;
                        cout << "-------------------------------------------------------------" << endl;
                        continue;
                    }
                    if (MIP_OF_SOLVE) {
                        bool warmstart(true);
                        solve_timecontinuous_mip_of(sol_sct[scenario], sol_routes[scenario], models, vehicles, locations,
                                                    transportation_times, number_locations, sol_routes[scenario],
                                                    blockings, TIME_LIMIT, start_scenario, warmstart);
                    }
                }
                if (scenario != 1 || !MIP_OF_SOLVE) {
                    int iter(1);
                    int threshold;
                    if (USE_ALGORITHMIC_FRAMEWORK) {
                        // with algorithmic framework
                        double root_node_lb = solve_root_node(scenario, models, vehicles, locations, number_locations,
                                                              number_tasks,
                                                              blockings, transportation_times, sol_routes[scenario],
                                                              scenario_horizon[scenario], sol_sct[scenario],
                                                              start_scenario, TIME_LIMIT,
                                                              MASTER_ACTIVITY_END, MAX_ROUTES_PER_VEHICLE);
                        threshold = (int) ceil(root_node_lb) + 1;
                    }
                    else {
                        // without algorithmic framework
                        threshold = (int) sol_sct[scenario];
                    }
                    while (threshold <= sol_sct[scenario]) {
                        cout << "Try threshold < " << threshold << endl;
                        int current_threshold(threshold);
                        int ub_escape(threshold - 1);
                        VecI best_routes(sol_routes[scenario]);
                        scenario_horizon[scenario] = vehicles.back().Arrival_time + current_threshold;
                        solve_branch_and_price(scenario, models, vehicles, locations, number_locations, number_tasks,
                                               blockings, transportation_times, best_routes, scenario_horizon[scenario], current_threshold, ub_escape,
                                               start_scenario, TIME_LIMIT, MASTER_ACTIVITY_END, BRANCHING, MAX_ROUTES_PER_VEHICLE);
                        if (current_threshold == threshold) {
                            ++threshold;
                            ++iter;
                            if (double(clock() - start_scenario) / CLOCKS_PER_SEC >= TIME_LIMIT)
                                break;
                        }
                        else {
                            sol_sct[scenario] = current_threshold;
                            sol_routes[scenario] = best_routes;
                            break;
                        }
                    }
                    iterations[scenario] = iter;
                }
            }
            if (double(clock() - start_scenario) / CLOCKS_PER_SEC < TIME_LIMIT) {
                string name = (*instance_it) + "_" + string_scenario;
                write_schedule_to_csv(name, models, vehicles, transportation_times, sol_routes[scenario], sol_sct[scenario]);
                if (scenario == 0) {
                    sol_sct[1] = sol_sct[0];
                    sol_routes[1] = sol_routes[0];
                    scenario_horizon[1] = scenario_horizon[0];
                    sol_sct[2] = sol_sct[0];
                    sol_routes[2] = sol_routes[0];
                    scenario_horizon[2] = scenario_horizon[0];
                    sol_sct[3] = sol_sct[0];
                    sol_routes[3] = sol_routes[0];
                    scenario_horizon[3] = scenario_horizon[0];
                }
                else if (scenario == 2) {
                    sol_sct[3] = sol_sct[2];
                    sol_routes[3] = sol_routes[2];
                    scenario_horizon[3] = scenario_horizon[2];
                }
            }
            else {
                sol_sct[scenario] = -1;
            }

            cout << "Scenario " << string_scenario << " solved: " << sol_sct[scenario] << endl;
            cout << "-------------------------------------------------------------" << endl;
            sol_times[scenario] = double(clock() - start_scenario) / CLOCKS_PER_SEC;
        }
        //---------------------------------------------------------------------------------------------------//
        double time_instance = double(clock() - start_instance) / CLOCKS_PER_SEC;
        cout << "---------------------------- End ----------------------------" << endl;
        cout << "Time to solve instance: " << time_instance << " seconds" << endl;
        cout << "Time to read data: " << time_read_data << " seconds" << endl;
        cout << "Time to preprocess: " << time_preprocessing << " seconds" << endl;
        cout << "Time NF scenario: " << sol_times[0] << " seconds" << endl;
        cout << "Time OF scenario: " << sol_times[1] << " seconds" << endl;
        cout << "Time RF scenario: " << sol_times[2] << " seconds" << endl;
        cout << "Time FF scenario: " << sol_times[3] << " seconds" << endl;
        cout << "-------------------------------------------------------------" << endl;
        cout << "Optimal SCT NF scenario: " << sol_sct[0] << "\t" << round((sol_sct[0] - sol_sct[0]) * 1.0 / sol_sct[0] * 1000) / 10.0 << "%" << endl;
        cout << "Optimal SCT OF scenario: " << sol_sct[1] << "\t" << round((sol_sct[0] - sol_sct[1]) * 1.0 / sol_sct[0] * 1000) / 10.0 << "%" << endl;
        cout << "Optimal SCT RF scenario: " << sol_sct[2] << "\t" << round((sol_sct[0] - sol_sct[2]) * 1.0 / sol_sct[0] * 1000) / 10.0 << "%" << endl;
        cout << "Optimal SCT FF scenario: " << sol_sct[3] << "\t" << round((sol_sct[0] - sol_sct[3]) * 1.0 / sol_sct[0] * 1000) / 10.0 << "%" << endl;
        cout << "-------------------------------------------------------------" << endl;
        // derive worker utilization
        write_solutions_to_csv(*instance_it, sol_sct, sol_times, iterations);
    }
    return 0;
}
//------------------------------------------------------------------------------------------------------------// 